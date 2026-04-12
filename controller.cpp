/*
 * =====================================================================================
 * ПРОЕКТ: Система управления рулонным пресс-подборщиком (Блок Пульта / Master)
 * ВЕРСИЯ: 1.0 (Release)
 * ОПИСАНИЕ: 
 * Данный код работает на Arduino Nano/Uno в кабине трактора. 
 * Он считывает состояние тумблеров и кнопок, выводит информацию на LCD 1602 (I2C) 
 * и отправляет команды управления по шине RS485 на блок пресса (Slave).
 * Реализованы: меню настроек таймингов, секундомер для калибровки и защита от зависаний.
 * =====================================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h> // Библиотека Сторожевого таймера (защита от зависаний)

// --- НАСТРОЙКИ ЭКРАНА ---
LiquidCrystal_I2C lcd(0x27, 16, 2); // Адрес дисплея обычно 0x27 или 0x3F

// =================================================================================
// РАСПИНОВКА ПУЛЬТА (Аппаратные пины)
// =================================================================================
#define PIN_SW_MODE      4    // Вход: Тумблер АВТО/РУЧНОЙ (Замыкается на GND)
#define PIN_SW_NET       2    // Вход: Тумблер СЕТКА/ШПАГАТ (Замыкается на GND)
#define PIN_BTN_SCREEN   3    // Вход: Кнопка переключения экранов/навигации
#define PIN_BTN_ACTION   11   // Вход: Кнопка действия (Пуск в ручном / Сброс / Изменение цифр)

#define PIN_RS485_RX     8    // RX-пин для связи с модулем MAX485
#define PIN_RS485_TX     9    // TX-пин для связи с модулем MAX485
#define PIN_RS485_EN     10   // Пин управления направлением передачи MAX485 (DE/RE)

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// =================================================================================
// СТРУКТУРЫ ДАННЫХ ДЛЯ ОБМЕНА ПО RS485
// =================================================================================

// Пакет данных, который Пульт (Master) отправляет Прессу (Slave)
struct MasterData { 
  bool doReset;         // Команда на экстренный сброс/возврат
  bool isManualMode;    // Статус: включен ли РУЧНОЙ режим
  bool isNetMode;       // Статус: выбран ли материал СЕТКА
  bool saveSettings;    // Команда: сохранить новые настройки в EEPROM пресса
  uint8_t timeoutDens;  // Настройка: Задержка плотности (сек)
  uint8_t timeoutStop;  // Настройка: Время на остановку трактора (сек)
  uint8_t timeoutNet;   // Настройка: Время подачи сетки (сек)
  uint8_t timeoutTwine; // Настройка: Время подачи шпагата (сек)
};
// Инициализация структуры начальными (безопасными) значениями
MasterData txData = {false, false, false, false, 1, 4, 2, 3}; 

// Пакет данных, который Пульт получает от Пресса (Slave)
struct SlaveData {
  uint8_t currentState;  // Текущий шаг работы пресса (0-8)
  uint16_t sessionBales; // Счетчик тюков за текущую смену
  uint32_t totalBales;   // Общий счетчик тюков (Тотал из EEPROM)
  uint8_t t_Dens;        // Текущая сохраненная настройка плотности
  uint8_t t_Stop;        // Текущая сохраненная настройка ожидания
  uint8_t t_Net;         // Текущая сохраненная настройка сетки
  uint8_t t_Twine;       // Текущая сохраненная настройка шпагата
};
SlaveData slaveData = {0, 0, 0, 1, 4, 2, 3}; 
SlaveData lastSlaveData = {255, 0, 0, 0, 0, 0, 0}; // Для отслеживания изменений и дебага

// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ СВЯЗИ ---
bool isConnected = false;        // Статус наличия связи с прессом
unsigned long lastPollTime = 0;  // Таймер последнего опроса шины RS485

// Кастомный символ для экрана: Иконка "Нет связи" (Перечеркнутый квадратик)
byte noLinkChar[8] = { 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b00000, 0b11111, 0b00000 };

// =================================================================================
// КЛАСС ДЛЯ РАБОТЫ С КНОПКАМИ (Антидребезг и обработка удержаний)
// =================================================================================
class Button {
  private:
    uint8_t pin; 
    unsigned long lastChange; 
    bool state; 
    bool lastReading; 
    bool stateChanged;
  public:
    // Конструктор
    Button(uint8_t p) : pin(p), lastChange(0), state(false), lastReading(false), stateChanged(false) {}
    
    // Инициализация пина (включаем внутреннюю подтяжку к 5V)
    void begin() { pinMode(pin, INPUT_PULLUP); }
    
    // Опрос кнопки (вызывать в loop)
    void update() {
      stateChanged = false;
      bool reading = !digitalRead(pin); // Кнопки замыкают на GND, поэтому инвертируем (!)
      if (reading != lastReading) lastChange = millis();
      // Фильтр дребезга контактов (50 мс)
      if ((millis() - lastChange) > 50) {
        if (state != reading) { state = reading; stateChanged = true; }
      }
      lastReading = reading;
    }
    
    bool isPressed() { return state; }                                         // Зажата ли кнопка сейчас
    bool justPressed() { return (state == true && stateChanged == true); }     // Нажата ли именно в этот цикл
    bool isHeldFor(uint32_t time) { return state && ((millis() - lastChange) >= time); } // Удерживается ли N миллисекунд
};

// Инициализация объектов кнопок
Button swMode(PIN_SW_MODE);
Button swNet(PIN_SW_NET);
Button btnScreen(PIN_BTN_SCREEN);
Button btnAction(PIN_BTN_ACTION);

// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ЛОГИКИ ПУЛЬТА ---
bool resetCommandSent = false;       
unsigned long lastDisplayUpdate = 0; 

// Управление экранами: 0 - Главный, 1 - Тотал, 2 - Настройки
uint8_t screenPage = 0;              
unsigned long screenTimer = 0;       

// Переменные для секундомера (режим калибровки)
unsigned long stopwatchStartTime = 0;
bool isStopwatchRunning = false;
uint16_t stoppedTimeSec = 0;
bool showStoppedTime = false;
uint8_t prevLoopState = 0;

// Переменные для Меню Настроек
unsigned long comboTimer = 0;
bool comboTriggered = false;
uint8_t settingIndex = 0; // Навигация: 0=Плотность, 1=Трактор, 2=Сетка, 3=Шпагат
uint8_t edit_t_Dens, edit_t_Stop, edit_t_Net, edit_t_Twine; // Временные переменные для редактирования
bool pendingSave = false; // Флаг: отправить пакет сохранения на пресс

// =================================================================================
// СТАРТОВАЯ НАСТРОЙКА
// =================================================================================
void setup() {
  // Настройка RS485
  pinMode(PIN_RS485_EN, OUTPUT); 
  digitalWrite(PIN_RS485_EN, LOW); // Режим ПРИЕМА по умолчанию
  rs485.begin(9600); 
  
  // Инициализация кнопок
  swMode.begin(); swNet.begin(); btnScreen.begin(); btnAction.begin();
  
  // Инициализация экрана
  lcd.init(); 
  lcd.backlight(); 
  lcd.createChar(0, noLinkChar); 
  
  // Заставка при включении
  lcd.setCursor(0, 0); lcd.print(F("BALER CONTROL")); 
  lcd.setCursor(0, 1); lcd.print(F("SYSTEM START..."));
  delay(1000); 
  lcd.clear();

  // Включаем аппаратный сторожевой таймер. 
  // Если программа зависнет более чем на 2 секунды, чип перезагрузится сам.
  wdt_enable(WDTO_2S); 
}

// =================================================================================
// ФУНКЦИЯ ОБЩЕНИЯ С ПРЕССОМ ПО RS485 (Master опрашивает Slave)
// =================================================================================
void pollSlave() {
  // 1. Подготавливаем актуальные данные о тумблерах для отправки
  txData.isManualMode = swMode.isPressed();
  txData.isNetMode = swNet.isPressed();

  // Если пользователь приказал сохранить настройки в меню
  if (pendingSave) { txData.saveSettings = true; }

  // Очистка приемного буфера перед передачей
  while (rs485.available()) rs485.read();
  
  // 2. ОТПРАВКА ЗАПРОСА
  digitalWrite(PIN_RS485_EN, HIGH); // Переключаем MAX485 на ПЕРЕДАЧУ
  delay(2); 
  rs485.write(0xBB); // Стартовый байт (заголовок пакета)
  
  uint8_t crc = 0; // Переменная для Контрольной Суммы (защита от помех)
  uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(MasterData); i++) { 
    rs485.write(ptr[i]); // Отправляем структуру побайтово
    crc ^= ptr[i];       // Считаем XOR-сумму
  }
  rs485.write(crc); // Отправляем контрольную сумму в конце
  rs485.flush();    // Ждем окончания физической передачи
  digitalWrite(PIN_RS485_EN, LOW); // Возвращаем MAX485 на ПРИЕМ

  // Сброс флага сохранения (отправляется только 1 раз)
  if (pendingSave) { txData.saveSettings = false; pendingSave = false; }

  // 3. ОЖИДАНИЕ ОТВЕТА ОТ ПРЕССА
  unsigned long waitStart = millis(); 
  bool replied = false;
  
  // Ждем начала ответа до 80 мс
  while (millis() - waitStart < 80) {
    if (rs485.available() > 0) {
      if (rs485.read() == 0xAA) { // Поймали стартовый байт ответа
        unsigned long pStart = millis();
        // Ждем пока весь пакет (структура + CRC) прилетит в буфер
        while (rs485.available() < sizeof(SlaveData) + 1) {
          if (millis() - pStart > 40) break; // Таймаут чтения пакета
        }
        
        // 4. ЧТЕНИЕ И ПРОВЕРКА ПАКЕТА
        if (rs485.available() >= sizeof(SlaveData) + 1) {
          uint8_t rCrc = 0; 
          uint8_t* rPtr = (uint8_t*)&slaveData;
          for (uint16_t i = 0; i < sizeof(SlaveData); i++) { 
            rPtr[i] = rs485.read(); 
            rCrc ^= rPtr[i]; 
          }
          // Сверяем контрольную сумму
          if (rCrc == rs485.read()) { 
            replied = true; // Пакет целый! Связь есть!
            txData.doReset = false; // Сбрасываем команду Reset после успешной доставки
          }
        }
        break; 
      }
    }
  }
  isConnected = replied; 
}

// =================================================================================
// ГЛАВНЫЙ ЦИКЛ ПРОГРАММЫ
// =================================================================================
void loop() {
  wdt_reset(); // "Гладим" сторожевую собаку, сообщая, что мы не зависли

  // Опрос физических кнопок
  swMode.update(); swNet.update(); btnScreen.update(); btnAction.update();

  // Отправка пакета по RS485 каждые 300 миллисекунд
  if (millis() - lastPollTime >= 300) { pollSlave(); lastPollTime = millis(); }

  // --- ЛОГИКА СЕКУНДОМЕРА ДЛЯ РУЧНОГО РЕЖИМА (Калибровка) ---
  if (swMode.isPressed()) {
    if (slaveData.currentState == 1 && prevLoopState == 0) {
      // Переход из "Набора массы" в "Остановку" - старт секундомера
      stopwatchStartTime = millis();
      isStopwatchRunning = true;
      showStoppedTime = false;
    }
    if (isStopwatchRunning && btnAction.justPressed()) {
      // Тракторист нажал пуск мотора - стоп секундомер
      stoppedTimeSec = (millis() - stopwatchStartTime) / 1000;
      isStopwatchRunning = false;
      showStoppedTime = true; // Оставляем цифру на экране
    }
    if (slaveData.currentState == 0) {
      // Сброс при начале нового цикла
      isStopwatchRunning = false;
      showStoppedTime = false;
    }
  } else {
    // В авторежиме секундомер выключен
    isStopwatchRunning = false;
    showStoppedTime = false;
  }
  prevLoopState = slaveData.currentState; // Запоминаем состояние для определения перехода

  // =================================================================================
  // ЛОГИКА МЕНЮ НАСТРОЕК (ВХОД И ВЫХОД)
  // =================================================================================
  // Зажаты обе кнопки (Экран + Действие) и есть связь с прессом
  if (btnScreen.isPressed() && btnAction.isPressed() && isConnected) {
    
    // Аппаратная "Защита от дурака": не пускаем в меню в ручном режиме
    if (swMode.isPressed()) {
      if (millis() - comboTimer >= 500) { 
        lcd.clear(); lcd.setCursor(0,0); lcd.print(F("PEREVEDI V AVTO!")); delay(1000); lcd.clear();
        comboTimer = millis();
      }
    } else {
      // Безопасный вход (Тумблер в АВТО)
      if (millis() - comboTimer >= 2000 && !comboTriggered) {
        comboTriggered = true; // Фиксатор срабатывания
        if (screenPage != 2) {
          // --- ВХОД В НАСТРОЙКИ ---
          screenPage = 2; settingIndex = 0;
          // Копируем настройки из памяти пресса во временные переменные для изменения
          edit_t_Dens = slaveData.t_Dens; edit_t_Stop = slaveData.t_Stop;
          edit_t_Net = slaveData.t_Net; edit_t_Twine = slaveData.t_Twine;
          lcd.clear(); lcd.setCursor(0,0); lcd.print(F("== NASTROYKI ==")); delay(1000); lcd.clear();
        } else {
          // --- ВЫХОД ИЗ НАСТРОЕК И СОХРАНЕНИЕ ---
          txData.timeoutDens = edit_t_Dens; txData.timeoutStop = edit_t_Stop;
          txData.timeoutNet = edit_t_Net; txData.timeoutTwine = edit_t_Twine;
          pendingSave = true; // Запускаем процесс отправки новых цифр на пресс
          
          screenPage = 0;
          lcd.clear(); lcd.setCursor(0,0); lcd.print(F("SOHRANENO V PRES")); delay(1000); lcd.clear();
        }
      }
    }
  } else {
    comboTimer = millis(); // Сброс таймера комбо, если кнопки отпущены
    comboTriggered = false;
  }

  // =================================================================================
  // ЛОГИКА НАВИГАЦИИ ПО ЭКРАНАМ И КНОПКАМ
  // =================================================================================
  if (screenPage == 2) { 
    // --- ВНУТРИ МЕНЮ НАСТРОЕК ---
    
    // Перелистывание параметров
    if (btnScreen.justPressed() && !btnAction.isPressed()) {
      settingIndex++; if (settingIndex > 3) settingIndex = 0; lcd.clear();
    }
    
    // Изменение значений
    if (btnAction.justPressed() && !btnScreen.isPressed()) {
      // Доп. защита: блокировка изменения, если тумблер переведен в Ручной во время настройки
      if (swMode.isPressed()) {
        lcd.setCursor(0, 1); lcd.print(F("PEREVEDI V AVTO!")); delay(1000); lcd.clear();
      } else {
        // Определяем, какую переменную мы сейчас меняем
        uint8_t *valPtr;
        if (settingIndex == 0) valPtr = &edit_t_Dens; else if (settingIndex == 1) valPtr = &edit_t_Stop;
        else if (settingIndex == 2) valPtr = &edit_t_Net; else if (settingIndex == 3) valPtr = &edit_t_Twine;

        // Изменение с помощью тумблера Сетка/Шпагат (+ / -)
        if (!swNet.isPressed()) { if (*valPtr < 20) (*valPtr)++; } // ШПАГАТ -> Увеличить
        else { if (*valPtr > 1) (*valPtr)--; }                     // СЕТКА -> Уменьшить
      }
    }
  } else {
    // --- ОБЫЧНЫЙ РЕЖИМ РАБОТЫ ---
    
    // Переключение экранов (Главный <-> Тотал)
    if (btnScreen.justPressed() && !btnAction.isPressed()) {
      screenPage = (screenPage == 0) ? 1 : 0; 
      screenTimer = millis(); 
      lcd.clear();                            
      updateDisplay(); 
      lastDisplayUpdate = millis();
    }

    // Обработка кнопки Сброса (только в АВТО режиме)
    if (!swMode.isPressed()) {
      if (btnAction.isHeldFor(1000) && !resetCommandSent && !btnScreen.isPressed()) {
        txData.doReset = true; resetCommandSent = true; 
        screenPage = 0; lcd.clear(); lcd.setCursor(0, 0); lcd.print(F(">> SBROS OK <<  ")); delay(1000); lcd.clear();
      }
      if (!btnAction.isPressed()) resetCommandSent = false;
    }
    
    // Автовозврат с экрана Тотала на Главный через 5 секунд
    if (screenPage == 1 && (millis() - screenTimer >= 5000)) { screenPage = 0; lcd.clear(); }
  }

  // Отрисовка экрана (не чаще 5 раз в секунду, чтобы не мерцало)
  if (millis() - lastDisplayUpdate > 200) { updateDisplay(); lastDisplayUpdate = millis(); }
}

// =================================================================================
// ФУНКЦИЯ ОТРИСОВКИ ИНТЕРФЕЙСА НА LCD
// =================================================================================
void updateDisplay() {
  if (screenPage == 0) {
    // === ГЛАВНЫЙ ЭКРАН ===
    lcd.setCursor(0, 0);
    
    // Если Ручной режим и зажата кнопка пуска (Аппаратный пуск)
    if (swMode.isPressed() && btnAction.isPressed()) { 
      lcd.print(F("PUSK MOTORA!   "));
    } else {
      // Текстовая расшифровка текущего состояния пресса
      switch (slaveData.currentState) {
        case 0: lcd.print(F("NABOR MASSY... ")); break; 
        case 1: lcd.print(swMode.isPressed() ? F("PLOTNOST:      ") : F("STOP TRAKTOR!  ")); break; 
        case 2: lcd.print(F("ZAHVAT...      ")); break; 
        case 3: lcd.print(F("OBVYAZKA...    ")); break; 
        case 4: lcd.print(F("OBREZKA...     ")); break; 
        case 5: lcd.print(F("OTKROY DVER!   ")); break; 
        case 6: lcd.print(F("SBOY! PROVER!  ")); break; 
        case 7: lcd.print(F("TEST REZHIM    ")); break; 
        case 8: lcd.print(F("VOZVRAT PLANKI ")); break; 
        default: lcd.print(F("N/A            ")); break; 
      }
    }

    // Отрисовка секундомера в правом верхнем углу (Поверх текста статуса)
    if (swMode.isPressed() && (isStopwatchRunning || showStoppedTime)) {
      lcd.setCursor(10, 0);
      char swBuf[6];
      sprintf(swBuf, "%2us  ", isStopwatchRunning ? (millis() - stopwatchStartTime) / 1000 : stoppedTimeSec);
      lcd.print(swBuf);
    }

    // Иконка отсутствия связи
    lcd.setCursor(15, 0);
    if (!isConnected) lcd.write(0); else lcd.print(F(" ")); 

    // Вторая строка: Статус тумблеров и счетчик смены
    lcd.setCursor(0, 1);
    char buffer[17]; char modesStr[9];
    strcpy(modesStr, swMode.isPressed() ? "[R]" : "[A]"); // Режим
    strcat(modesStr, swNet.isPressed() ? "[SET]" : "[SHP]"); // Материал
    sprintf(buffer, "%-8s%8u", modesStr, slaveData.sessionBales); // Форматированный вывод
    lcd.print(buffer); 

  } else if (screenPage == 1) {
    // === ЭКРАН ТОТАЛА ===
    lcd.setCursor(0, 0); lcd.print(F("TOTAL RULONOV:  ")); 
    lcd.setCursor(15, 0); if (!isConnected) lcd.write(0);
    lcd.setCursor(0, 1);
    char buffer[17];
    // Печать 32-битного числа (long unsigned)
    sprintf(buffer, "      %-10lu", slaveData.totalBales); 
    lcd.print(buffer);

  } else if (screenPage == 2) {
    // === ЭКРАН МЕНЮ НАСТРОЕК ===
    lcd.setCursor(0, 0);
    lcd.print(F("SET: "));
    // Имя текущего параметра
    switch (settingIndex) {
      case 0: lcd.print(F("PLOTNOST  ")); break;
      case 1: lcd.print(F("TRACTOR   ")); break;
      case 2: lcd.print(F("SETKA     ")); break;
      case 3: lcd.print(F("SHPAGAT   ")); break;
    }
    lcd.setCursor(0, 1); lcd.print(F("VREMYA: "));
    // Значение текущего параметра
    uint8_t val = 0;
    if (settingIndex == 0) val = edit_t_Dens; else if (settingIndex == 1) val = edit_t_Stop;
    else if (settingIndex == 2) val = edit_t_Net; else if (settingIndex == 3) val = edit_t_Twine;
    char buffer[10]; sprintf(buffer, "%2u sek  ", val); lcd.print(buffer);
  }
}
