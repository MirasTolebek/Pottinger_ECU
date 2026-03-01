#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Инициализация дисплея I2C. Адрес 0x27, размер 16 столбцов, 2 строки.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =================================================================================
// РАСПИНОВКА (PINS) ПУЛЬТА - Физические подключения к плате
// =================================================================================
#define PIN_SW_MODE      4    // Вход: Тумблер АВТО/РУЧНОЙ (Замкнут на GND = АВТО, Разомкнут = РУЧНОЙ)
#define PIN_SW_NET       5    // Вход: Тумблер СЕТКА/ШПАГАТ (Замкнут на GND = СЕТКА, Разомкнут = ШПАГАТ)
#define PIN_BTN_SCREEN   6    // Вход: Кнопка переключения экранов (Замыкает на GND)
#define PIN_BTN_ACTION   7    // Вход: Кнопка Сброса (Авто-режим) / Ручной пуск мотора (Ручной режим). Замыкает на GND.

// ПИНЫ ДЛЯ МОДУЛЯ СВЯЗИ MAX485 (Используем цифровые, т.к. на пульте их много свободных)
#define PIN_RS485_RX     8    // Подключается к ножке RO на модуле MAX485
#define PIN_RS485_TX     9    // Подключается к ножке DI на модуле MAX485
#define PIN_RS485_EN     10   // Подключается к спаянным вместе ножкам DE и RE на MAX485

// Инициализация программного порта связи
SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// =================================================================================
// СТРУКТУРЫ ДАННЫХ ДЛЯ ОБЩЕНИЯ ПО RS485 (Точная копия структур из блока пресса)
// =================================================================================

// Пакет ДЛЯ отправки на Пресс (Master -> Slave)
struct MasterData {
  bool doReset; // Флаг команды "Сделай сброс"
};
MasterData txData = {false};

// Пакет ОТ Пресса (Slave -> Master)
struct SlaveData {
  uint8_t currentState;  // Шаг работы автомата на прессе
  uint16_t totalBales;   // Счетчик за все время
  uint16_t sessionBales; // Счетчик за смену
};
SlaveData slaveData = {0, 0, 0}; // При включении пульта нули, пока не придут данные от пресса

// Глобальные переменные статуса связи
bool isConnected = false;        // true, если пресс отвечает на запросы
unsigned long lastPollTime = 0;  // Таймер регулярных опросов пресса

// =================================================================================
// КАСТОМНАЯ ИКОНКА "НЕТ СВЯЗИ" ДЛЯ ДИСПЛЕЯ (Рисуем попиксельно)
// =================================================================================
// 1 = закрашенный пиксель, 0 = пустой. Формирует перечеркнутый крестик X с подчеркиванием.
byte noLinkChar[8] = {
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10001,
  0b00000,
  0b11111, // Подчеркивание
  0b00000
};

// =================================================================================
// КЛАСС ДЛЯ РАБОТЫ С КНОПКАМИ ПУЛЬТА (Защита от дребезга контактов)
// =================================================================================
class Button {
  private:
    uint8_t pin;
    unsigned long lastChange;
    bool state;
    bool lastReading;
    bool stateChanged;

  public:
    Button(uint8_t p) : pin(p), lastChange(0), state(false), lastReading(false), stateChanged(false) {}
    
    void begin() { 
      // Жестко включаем внутреннюю подтяжку (5V). Кнопки и тумблеры ДОЛЖНЫ замыкать на массу (GND)
      pinMode(pin, INPUT_PULLUP); 
    }
    
    void update() {
      stateChanged = false;
      // Читаем сигнал. Так как кнопка замыкает на GND, LOW означает "Нажато".
      // Оператор ! (НЕ) переворачивает LOW в true (логика "Кнопка нажата").
      bool reading = !digitalRead(pin); 
      
      if (reading != lastReading) lastChange = millis();
      
      // Антидребезг 50 мс
      if ((millis() - lastChange) > 50) {
        if (state != reading) {
          state = reading;
          stateChanged = true;
        }
      }
      lastReading = reading;
    }
    
    bool isPressed() { return state; } // Удерживается ли кнопка прямо сейчас
    bool justPressed() { return (state == true && stateChanged == true); } // Было ли совершено нажатие в этот самый момент (Клик)
    bool isHeldFor(uint32_t time) { return state && ((millis() - lastChange) >= time); } // Удерживается ли дольше заданного времени
};

// Создание объектов кнопок
Button swMode(PIN_SW_MODE);
Button swNet(PIN_SW_NET);
Button btnScreen(PIN_BTN_SCREEN);
Button btnAction(PIN_BTN_ACTION);

// Глобальные переменные интерфейса
bool resetCommandSent = false;       // Блокировка от спама командами сброса
unsigned long lastDisplayUpdate = 0; // Таймер обновления дисплея (чтобы не мерцал)

uint8_t screenPage = 0;              // Текущий экран: 0 = Главный (Смена), 1 = Статистика (Тотал)
unsigned long screenTimer = 0;       // Таймер для автовозврата на главный экран

// =================================================================================
// НАСТРОЙКА ПРИ ВКЛЮЧЕНИИ (SETUP)
// =================================================================================
void setup() {
  Serial.begin(115200);
  
  // Инициализация RS485 (Пульт всегда начинает с прослушивания)
  pinMode(PIN_RS485_EN, OUTPUT);
  digitalWrite(PIN_RS485_EN, LOW); // Режим ПРИЕМ
  rs485.begin(9600); // Скорость должна строго совпадать с блоком пресса
  
  // Инициализация кнопок
  swMode.begin();
  swNet.begin();
  btnScreen.begin();
  btnAction.begin();
  
  // Инициализация дисплея
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, noLinkChar); // Загружаем кастомную иконку "Нет связи" в память дисплея (слот 0)
  
  // Заставка при запуске
  lcd.setCursor(0, 0);
  lcd.print(F("BALER CONTROL"));
  lcd.setCursor(0, 1);
  lcd.print(F("SYSTEM START..."));
  delay(1000); // Даем время блоку пресса тоже загрузиться
  lcd.clear();
}

// =================================================================================
// ФУНКЦИЯ ОБЩЕНИЯ С БЛОКОМ ПРЕССА (Пульт - это Мастер, он сам инициирует диалог)
// =================================================================================
void pollSlave() {
  // 1. Очищаем буфер от случайного мусора перед отправкой запроса
  while (rs485.available()) rs485.read();

  // 2. ОТПРАВКА ЗАПРОСА
  digitalWrite(PIN_RS485_EN, HIGH); // Включаем режим ПЕРЕДАЧА
  delay(2); // Стабилизация линии
  
  rs485.write(0xBB); // Отправляем СТАРТОВЫЙ БАЙТ Мастера (чтобы пресс понял, что с ним говорят)
  
  // Отправляем структуру с командами и считаем CRC
  uint8_t crc = 0;
  uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(MasterData); i++) {
    rs485.write(ptr[i]);
    crc ^= ptr[i];
  }
  rs485.write(crc);

  rs485.flush(); // Ждем завершения физической отправки
  digitalWrite(PIN_RS485_EN, LOW); // Возвращаемся в режим ПРИЕМ

  // 3. ОЖИДАНИЕ ОТВЕТА ОТ ПРЕССА
  unsigned long waitStart = millis();
  bool replied = false;
  
  // Даем прессу максимум 100 мс на ответ. Если ответа нет - связь потеряна.
  while (millis() - waitStart < 100) {
    if (rs485.available() > 0) {
      if (rs485.read() == 0xAA) { // Поймали стартовый байт ответа от пресса
        delay(10); // Ждем, пока долетит вся структура ответа
        
        // Проверяем полноту пакета
        if (rs485.available() >= sizeof(SlaveData) + 1) {
          uint8_t rCrc = 0;
          uint8_t* rPtr = (uint8_t*)&slaveData;
          
          // Читаем данные напрямую в глобальную структуру slaveData
          for (uint16_t i = 0; i < sizeof(SlaveData); i++) {
            rPtr[i] = rs485.read();
            rCrc ^= rPtr[i];
          }
          
          // Сверяем контрольную сумму
          if (rCrc == rs485.read()) {
            replied = true; // Пакет целый, связь есть!
            txData.doReset = false; // Если мы слали команду Сброс - она успешно доставлена, снимаем флаг
          }
        }
        break; // Ответ получен (или забракован), выходим из цикла ожидания
      }
    }
  }
  
  isConnected = replied; // Обновляем глобальный статус связи (для иконки на экране)
}

// =================================================================================
// ГЛАВНЫЙ РАБОЧИЙ ЦИКЛ ПУЛЬТА (LOOP)
// =================================================================================
void loop() {
  // 1. Опрос всех кнопок и тумблеров
  swMode.update();
  swNet.update();
  btnScreen.update();
  btnAction.update();

  // 2. ОБЩЕНИЕ ПО RS485 (Дергаем пресс каждые 300 мс)
  if (millis() - lastPollTime >= 300) {
    pollSlave();
    lastPollTime = millis();
  }

  // --- ЛОГИКА КНОПКИ 1: ЭКРАН (Работает всегда, только листает меню) ---
  if (btnScreen.justPressed()) {
    screenPage = (screenPage == 0) ? 1 : 0; // Переключение 0->1 или 1->0
    screenTimer = millis();                 // Засекаем время открытия статистики
    lcd.clear();                            // Очищаем артефакты старого экрана
  }

  // --- ЛОГИКА КНОПКИ 2: ДЕЙСТВИЕ (Зависит от тумблера АВТО/РУЧНОЙ) ---
  if (swMode.isPressed()) {
    // === РУЧНОЙ РЕЖИМ (Тумблер вниз/вверх) ===
    // В этом режиме кнопка физически запускает мотор в обход Ардуино.
    // Поэтому по RS485 ничего не шлем. Индикация "PUSK MOTORA!" обрабатывается в блоке экрана.
  } else {
    // === АВТО РЕЖИМ ===
    // Кнопка работает как запрос на АВАРИЙНЫЙ СБРОС (защита: нужно удерживать 1 секунду)
    if (btnAction.isHeldFor(1000) && !resetCommandSent) {
      Serial.println(F("SEND RS485: RESET COMMAND!"));
      txData.doReset = true; // Взводим курок! Команда улетит при следующем вызове pollSlave()
      resetCommandSent = true; // Блокировка от повторной отправки, пока не отпустят кнопку
      
      screenPage = 0; // Принудительно выкидываем на главный экран
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F(">> SBROS OK <<  ")); // Визуальное подтверждение для тракториста
      delay(1000); 
      lcd.clear();
    }
    // Снимаем блокировку, если кнопку отпустили
    if (!btnAction.isPressed()) {
      resetCommandSent = false;
    }
  }

  // Авто-возврат со страницы Статистики на Главный экран (Через 5 секунд)
  if (screenPage == 1 && (millis() - screenTimer >= 5000)) {
    screenPage = 0;
    lcd.clear();
  }

  // 3. ОТРИСОВКА ЭКРАНА (Не чаще 5 раз в секунду, чтобы не было мерцания)
  if (millis() - lastDisplayUpdate > 200) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

// =================================================================================
// ИНТЕРФЕЙС ЭКРАНА (Отрисовка всех меню)
// =================================================================================
void updateDisplay() {
  
  // === ЭКРАН 0: ГЛАВНЫЙ (Рабочий режим) ===
  if (screenPage == 0) {
    
    // --- ПЕРВАЯ СТРОКА: Статус ---
    lcd.setCursor(0, 0);
    
    if (swMode.isPressed()) { 
      // Если включен РУЧНОЙ режим
      if (btnAction.isPressed()) lcd.print(F("PUSK MOTORA!   "));
      else lcd.print(F("GOTOV K RABOTE "));
    } else {
      // Если включен АВТО режим.
      // Расшифровываем цифру currentState (присланную от пресса) в понятный текст (макс. 15 символов)
      switch (slaveData.currentState) {
        case 0: lcd.print(F("NABOR MASSY... ")); break; // WAIT_DENSITY
        case 1: lcd.print(F("STOP TRAKTOR!  ")); break; // WAIT_TRACTOR
        case 2: lcd.print(F("ZAHVAT...      ")); break; // WAIT_START_SENSOR
        case 3: lcd.print(F("OBVYAZKA...    ")); break; // MOTOR_RUNNING_TIMER
        case 4: lcd.print(F("OBREZKA...     ")); break; // WAIT_END_SENSOR
        case 5: lcd.print(F("OTKROY DVER!   ")); break; // WAIT_DOOR
        case 6: lcd.print(F("SBOY! PROVER!  ")); break; // ERROR_STATE
        case 7: lcd.print(F("TEST REZHIM    ")); break; // TEST_MODE
        case 8: lcd.print(F("VOZVRAT PLANKI ")); break; // RETURN_TO_HOME
        default: lcd.print(F("N/A            ")); break; // Если пришел мусор
      }
    }

    // ЗНАЧОК СВЯЗИ (Рисуется самым последним символом в верхней строке, позиция 15)
    lcd.setCursor(15, 0);
    if (!isConnected) {
      lcd.write(0); // Вывод кастомного символа "Крестик" из памяти (слот 0)
    } else {
      lcd.print(F(" ")); // Если связь есть - затираем крестик пробелом
    }

    // --- ВТОРАЯ СТРОКА: Режимы и счетчик за смену ---
    lcd.setCursor(0, 1);
    
    // Формируем красивую строку, выровненную по краям экрана. Пример: "[A][SET]      124"
    char buffer[17]; // Буфер для всей строки (16 символов + нуль-терминатор)
    char modesStr[9]; // Буфер для куска режимов (например "[A][SET]")
    
    // Собираем кусок режимов:
    strcpy(modesStr, swMode.isPressed() ? "[R]" : "[A]"); // Копируем первую часть [R] или [A]
    strcat(modesStr, swNet.isPressed() ? "[SET]" : "[SHP]"); // Приклеиваем вторую часть
    
    // Собираем всё в одну строку:
    // %-8s : Вставить текст из modesStr, выровнять по левому краю (ширина 8 символов). Минус - это выравнивание влево.
    // %8u  : Вставить число sessionBales, выровнять по правому краю (ширина 8 символов). Заполнит пустоты пробелами.
    sprintf(buffer, "%-8s%8u", modesStr, slaveData.sessionBales);
    lcd.print(buffer); // Выводим готовую строку на экран за один раз
  } 
  
  // === ЭКРАН 1: ТОТАЛ (Статистика за всю жизнь) ===
  else if (screenPage == 1) {
    lcd.setCursor(0, 0);
    lcd.print(F("TOTAL RULONOV:  ")); 
    
    // Значок связи (чтобы было видно потерю связи даже в меню статистики)
    lcd.setCursor(15, 0);
    if (!isConnected) lcd.write(0);
    
    lcd.setCursor(0, 1);
    char buffer[17];
    // Форматируем число с отступами: 6 пробелов + число, выровненное по левому краю (на 10 позиций)
    sprintf(buffer, "      %-10u", slaveData.totalBales);
    lcd.print(buffer);
  }
}
