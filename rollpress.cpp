#include <SoftwareSerial.h>

// =================================================================================
// РАСПИНОВКА (PINS) - Физические подключения к плате
// =================================================================================
#define PIN_DENSITY        2    // Вход: Концевик "Плотность рулона" (через оптрон)
#define PIN_S_START        3    // Вход: Концевик "Начало обвязки" (через оптрон)
#define PIN_S_END          4    // Вход: Концевик "Конец обвязки" (через оптрон)
#define PIN_DOOR           5    // Вход: Датчик "Дверь камеры открыта" (через оптрон)
#define PIN_RELAY_TWINE    6    // Выход: Реле мотора ШПАГАТА (через резистор 120 Ом)
#define PIN_RELAY_SOUND    7    // Выход: Реле звукового сигнала/гудка (через резистор 120 Ом)
#define PIN_RESET          8    // Вход: Кнопка "Сброс/Авария" на корпусе блока (через RC-фильтр)
#define PIN_RELAY_LIGHT    9    // Выход: Реле проблескового маячка (через резистор 120 Ом)
#define PIN_RELAY_NET      10   // Выход: Реле мотора СЕТКИ (через резистор 120 Ом)
#define PIN_SWITCH_NET     11   // Вход: Тумблер выбора материала на корпусе (Сетка/Шпагат)

// ПИНЫ ДЛЯ МОДУЛЯ СВЯЗИ MAX485 (Используем аналоговые пины как цифровые, чтобы не занимать USB)
#define PIN_RS485_RX       A0   // Подключается к ножке RO на модуле MAX485
#define PIN_RS485_TX       A1   // Подключается к ножке DI на модуле MAX485
#define PIN_RS485_EN       A2   // Подключается к спаянным вместе ножкам DE и RE на модуле MAX485

// Инициализация программного последовательного порта для связи с пультом
SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// =================================================================================
// ГИБКИЕ НАСТРОЙКИ (ТАЙМИНГИ И ЛОГИКА) - Меняются здесь при наладке в поле
// =================================================================================
struct Config {
  const uint32_t plotnostLozh = 1000;       // Задержка (мс) от ложных срабатываний датчика плотности на кочках
  const uint32_t timeForStopTraktor = 4000; // Время (мс), которое дается трактористу на остановку после гудка
  const uint32_t timeoutMotorMax = 10000;   // Максимальное время (мс) работы мотора до аварии (если не сработал концевик Старт)
  const uint32_t timerTwineRun = 3000;      // Время (мс) работы мотора ШПАГАТА (от Старта до отсечки таймером)
  const uint32_t timerNetRun = 2000;        // Время (мс) работы мотора СЕТКИ (от Старта до отсечки таймером)
  const uint32_t timeoutEndSensor = 15000;  // Максимальное время (мс) ожидания отрезки (концевика Конец) до аварии
  const uint32_t timeoutReturnHome = 10000; // Максимальное время (мс) на возврат механизма в исходное положение до аварии
  const uint32_t debounce = 50;             // Время (мс) антидребезга для всех кнопок и концевиков

  // Настройка управляющих уровней для реле (зависит от купленных модулей реле)
  const bool motorActiveHigh = true;  // true - реле моторов включаются высоким уровнем (5V)
  const bool soundActiveHigh = false; // false - реле гудка включается низким уровнем (GND/0V)
  const bool lightActiveHigh = true;  // true - реле маячка включается высоким уровнем (5V)
};
Config cfg;

// =================================================================================
// СТРУКТУРЫ ДАННЫХ ДЛЯ ОБЩЕНИЯ ПО RS485 (Должны быть одинаковыми на обоих устройствах)
// =================================================================================

// Пакет, который мы получаем от Пульта (Master -> Slave)
struct MasterData {
  bool doReset; // Флаг: если true, пульт приказал сделать аварийный сброс
};
MasterData rxData;

// Пакет, который мы отправляем Пульту (Slave -> Master)
struct SlaveData {
  uint8_t currentState;  // Текущий шаг работы пресса (для отображения текста на экране)
  uint16_t totalBales;   // Общее количество рулонов за всю жизнь
  uint16_t sessionBales; // Количество рулонов за текущую смену
};
SlaveData txData = {0, 0, 0}; // Начальные значения при включении питания

// =================================================================================
// КЛАССЫ И ПОДПРОГРАММЫ (ДЛЯ УДОБСТВА УПРАВЛЕНИЯ ЖЕЛЕЗОМ)
// =================================================================================

// Класс для обработки кнопок и концевиков с защитой от дребезга
class Sensor {
  private:
    uint8_t pin;
    bool invertLogic; // Если true, то замыкание на GND считается нажатием (LOW = Pressed)
    unsigned long lastChange;
    unsigned long stateChangeTime;
    bool state;
    bool lastReading;
    bool stateChanged;

  public:
    // Конструктор: запоминаем пин и тип логики
    Sensor(uint8_t p, bool inv = true) : pin(p), invertLogic(inv), lastChange(0), stateChangeTime(0), state(false), lastReading(false), stateChanged(false) {}
    
    // Инициализация пина (если логика инверсная, включаем внутренний подтягивающий резистор)
    void begin() { pinMode(pin, invertLogic ? INPUT_PULLUP : INPUT); }
    
    // Функция регулярного опроса пина (нужно вызывать в loop)
    void update() {
      stateChanged = false; 
      bool reading = digitalRead(pin);
      if (invertLogic) reading = !reading; // Переворачиваем логику для удобства (нажато = true)
      
      if (reading != lastReading) lastChange = millis(); // Засекаем время изменения состояния
      
      // Если состояние не менялось дольше времени антидребезга (50мс)
      if ((millis() - lastChange) >= cfg.debounce) {
        if (state != reading) {
          state = reading;
          stateChanged = true;
          stateChangeTime = millis(); // Запоминаем время, когда состояние окончательно изменилось
        }
      }
      lastReading = reading;
    }
    bool isPressed() { return state; } // Проверка: нажата ли кнопка сейчас?
    bool justPressed() { return (state == true && stateChanged == true); } // Проверка: кнопку только что нажали?
    bool isHeldFor(uint32_t time) { return state && ((millis() - stateChangeTime) >= time); } // Проверка: кнопка удерживается заданное время?
};

// Класс для управления звуковым сигналом (гудком) без использования delay()
class Signaler {
  private:
    uint8_t pin;
    int beepsLeft = 0; // Сколько смен состояний (вкл/выкл) осталось отработать
    unsigned long lastToggle = 0;
    bool isRelayOn = false;
    uint32_t duration; // Длительность одного "пика"
    
    void turnOn()  { digitalWrite(pin, cfg.soundActiveHigh ? HIGH : LOW); }
    void turnOff() { digitalWrite(pin, cfg.soundActiveHigh ? LOW : HIGH); }

  public:
    Signaler(uint8_t p) : pin(p) {}
    void begin() { pinMode(pin, OUTPUT); turnOff(); }
    
    // Запуск серии гудков. Параметры: количество гудков, длительность гудка в мс
    void play(int count, uint32_t dur = 300) { 
      beepsLeft = count * 2; // Умножаем на 2, т.к. 1 гудок = 1 вкл + 1 выкл
      duration = dur; 
      isRelayOn = true; 
      turnOn(); 
      lastToggle = millis(); 
      beepsLeft--;
    }
    
    // Функция регулярного обновления (переключает реле по таймеру)
    void update() {
      if (beepsLeft > 0 && (millis() - lastToggle >= duration)) {
        isRelayOn = !isRelayOn;
        if (isRelayOn) turnOn(); else turnOff();
        lastToggle = millis(); 
        beepsLeft--;
      }
    }
    bool isBusy() { return beepsLeft > 0; } // Проверка: гудок сейчас в процессе работы?
};

// Класс для управления проблесковым маячком
class LightController {
  private:
    uint8_t pin;
    uint8_t currentMode; // 0=Выкл, 1=Вкл постоянно, 2=Быстрое мигание, 3=Медленное мигание
    unsigned long lastToggle;
    bool state;
    void turnOn()  { digitalWrite(pin, cfg.lightActiveHigh ? HIGH : LOW); }
    void turnOff() { digitalWrite(pin, cfg.lightActiveHigh ? LOW : HIGH); }

  public:
    LightController(uint8_t p) : pin(p), currentMode(0), lastToggle(0), state(false) {}
    void begin() { pinMode(pin, OUTPUT); turnOff(); }
    
    // Установка режима работы маячка
    void setMode(uint8_t mode) {
      currentMode = mode;
      if (mode == 0) { state = false; turnOff(); }
      else if (mode == 1) { state = true; turnOn(); }
    }
    
    // Функция регулярного обновления (для мигающих режимов)
    void update() {
      if (currentMode == 2) { 
        if (millis() - lastToggle >= 300) { state = !state; if (state) turnOn(); else turnOff(); lastToggle = millis(); }
      } else if (currentMode == 3) { 
        if (millis() - lastToggle >= 1000) { state = !state; if (state) turnOn(); else turnOff(); lastToggle = millis(); }
      }
    }
};

// =================================================================================
// СОЗДАНИЕ ОБЪЕКТОВ И ПЕРЕМЕННЫХ
// =================================================================================

// Инициализация датчиков. Все концевики и кнопки замыкают на GND (логика = true).
// Только датчик двери работает напрямую (логика = false), если это спец. датчик.
Sensor densSensor(PIN_DENSITY, true); 
Sensor startSensor(PIN_S_START, true);
Sensor endSensor(PIN_S_END, true);
Sensor doorSensor(PIN_DOOR, false); 
Sensor resetBtn(PIN_RESET, true); 

Signaler horn(PIN_RELAY_SOUND);
LightController beacon(PIN_RELAY_LIGHT);

// Перечисление всех возможных состояний пресса (Конечный автомат)
enum BalerState { 
  WAIT_DENSITY,         // 0: Ожидание набора массы (штатная работа)
  WAIT_TRACTOR,         // 1: Рулон набран, ждем остановки трактора
  WAIT_START_SENSOR,    // 2: Мотор запущен, ждем концевик "Начало обвязки"
  MOTOR_RUNNING_TIMER,  // 3: Идет обвязка (работает таймер времени мотора)
  WAIT_END_SENSOR,      // 4: Ждем концевик "Отрезка завершена"
  WAIT_DOOR,            // 5: Ждем открытия и закрытия задней камеры
  ERROR_STATE,          // 6: Авария! Что-то пошло не так
  TEST_MODE,            // 7: Режим проверки концевиков (диагностика)
  RETURN_TO_HOME        // 8: Режим принудительного возврата планки на базу
};
BalerState currentState = WAIT_DENSITY; // Состояние при включении питания

unsigned long stateTimer = 0;        // Универсальный таймер для отсчета задержек внутри состояний
unsigned long testModeStartTime = 0; // Таймер для автовыхода из режима диагностики
bool doorWasOpened = false;          // Флаг защиты: дверь должна быть сначала открыта, потом закрыта

uint8_t resetClicks = 0;             // Счетчик быстрых нажатий кнопки сброса (для спец-режимов)
unsigned long lastResetClickTime = 0;

// Чтение тумблера: Замкнут на GND = режим "Сетка" (true), разомкнут = "Шпагат" (false)
bool isNetModeActive() { return !digitalRead(PIN_SWITCH_NET); }

// Функция включения нужного мотора в зависимости от выбранного материала
void motorOn() { 
  if (isNetModeActive()) digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? HIGH : LOW);
  else digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? HIGH : LOW);
}

// Функция принудительного отключения обоих моторов
void motorOff() { 
  digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? LOW : HIGH);
  digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? LOW : HIGH);
}

// Универсальная функция аварийного сброса (вызывается пультом или кнопкой на блоке)
void executeEmergencyReset() {
  if (currentState != WAIT_DENSITY && currentState != TEST_MODE) {  
    Serial.println(F("--- CYCLE ABORTED (RESET) ---"));
    motorOff();                            // Глушим моторы
    horn.play(1, 600);                     // Длинный одиночный гудок
    beacon.setMode(0);                     // Гасим маяк
    doorWasOpened = doorSensor.isPressed();// Сбрасываем флаг двери
    currentState = WAIT_DENSITY;           // Возвращаемся в самое начало
  }
}

// =================================================================================
// ФУНКЦИИ СВЯЗИ ПО ПРОТОКОЛУ RS485
// =================================================================================

// Функция сборки и отправки ответного пакета Пульту (Вызывается только после получения запроса)
void sendRS485Reply() {
  digitalWrite(PIN_RS485_EN, HIGH); // Переводим чип MAX485 в режим ПЕРЕДАЧИ (TX)
  delay(2); // Обязательная микро-зауза, чтобы линия успела переключиться физически

  txData.currentState = currentState; // Обновляем текущий статус в пакете
  
  rs485.write(0xAA); // Отправляем СТАРТОВЫЙ БАЙТ (Маячок для пульта, что данные начались)
  
  // Отправка самой структуры побайтово с одновременным расчетом контрольной суммы (CRC)
  uint8_t crc = 0;
  uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(SlaveData); i++) {
    rs485.write(ptr[i]);
    crc ^= ptr[i]; // Операция XOR для вычисления простейшего CRC
  }
  rs485.write(crc); // Отправляем байт контрольной суммы в конце

  rs485.flush(); // Блокируем код, пока последний байт физически не улетит в провод
  digitalWrite(PIN_RS485_EN, LOW); // Сразу возвращаем чип MAX485 обратно в режим ПРИЕМА (RX)
}

// Функция прослушивания линии (Ждем запросов от Пульта)
void listenRS485() {
  if (rs485.available() > 0) { // Если в буфере есть данные
    if (rs485.read() == 0xBB) { // Проверяем: это стартовый байт Пульта?
      delay(5); // Ждем 5мс, пока остальные байты долетят по проводу в буфер
      
      // Проверяем, пришел ли пакет целиком (Размер структуры + 1 байт CRC)
      if (rs485.available() >= sizeof(MasterData) + 1) { 
        uint8_t crc = 0;
        uint8_t* ptr = (uint8_t*)&rxData;
        
        // Читаем пакет и параллельно считаем его контрольную сумму
        for (uint16_t i = 0; i < sizeof(MasterData); i++) {
          ptr[i] = rs485.read();
          crc ^= ptr[i];
        }
        
        uint8_t receivedCrc = rs485.read(); // Читаем CRC, которую прислал пульт
        
        // Если наша посчитанная CRC совпала с присланной - значит помех в кабеле не было
        if (crc == receivedCrc) {
          
          // Обрабатываем команды от пульта
          if (rxData.doReset) {
            executeEmergencyReset(); // Пульт прислал команду СБРОС!
          }
          
          // Отправляем ответ пульту со своей статистикой
          sendRS485Reply();
        }
      }
    }
  }
}

// =================================================================================
// НАСТРОЙКА ПРИ ВКЛЮЧЕНИИ ПИТАНИЯ (SETUP)
// =================================================================================
void setup() {
  Serial.begin(115200); // Для отладки через USB (когда ноутбук подключен)
  
  // Настройка пина управления MAX485
  pinMode(PIN_RS485_EN, OUTPUT);
  digitalWrite(PIN_RS485_EN, LOW); // По умолчанию мы всегда СЛУШАЕМ (Receive)
  
  // Запуск SoftwareSerial. 9600 бод - самая пуленепробиваемая скорость для длинных проводов в тракторе.
  rs485.begin(9600); 
  
  // Инициализация датчиков и кнопок
  densSensor.begin(); startSensor.begin(); endSensor.begin(); doorSensor.begin(); resetBtn.begin();
  pinMode(PIN_SWITCH_NET, INPUT_PULLUP); // Подтяжка для тумблера Сетка/Шпагат
  
  // Инициализация выходов
  horn.begin(); beacon.begin();
  pinMode(PIN_RELAY_TWINE, OUTPUT);
  pinMode(PIN_RELAY_NET, OUTPUT);
  motorOff(); // Гарантированно глушим моторы при старте
  
  Serial.println(F("--- BALER UNIT START ---"));
  beacon.setMode(0); // Выключаем маяк
}

// =================================================================================
// ГЛАВНЫЙ РАБОЧИЙ ЦИКЛ (LOOP) - Крутится бесконечно
// =================================================================================
void loop() {
  // 1. Опрос всех входов (обновление состояний с антидребезгом)
  densSensor.update(); startSensor.update(); endSensor.update(); doorSensor.update(); resetBtn.update();
  // 2. Обновление таймеров выходов (мигалки и гудки)
  horn.update(); beacon.update(); 
  
  // 3. ПРОВЕРКА ЛИНИИ СВЯЗИ RS485
  listenRS485();

  // --- ОБРАБОТКА КНОПКИ СБРОСА НА САМОМ БЛОКЕ ---
  bool isResetJustPressed = resetBtn.justPressed();

  // Логика 1: Вход в режим диагностики (Удержание кнопки 10 секунд)
  if (resetBtn.isHeldFor(10000) && currentState != TEST_MODE) {
    motorOff(); 
    horn.play(1, 100); 
    beacon.setMode(3); // Включаем медленное мигание маячка                 
    currentState = TEST_MODE; 
    testModeStartTime = millis();
    resetClicks = 0; 
  }

  // Логика 2: Обработка коротких нажатий (Кликов)
  if (isResetJustPressed && currentState != TEST_MODE) {
    // Считаем клики (если между нажатиями менее 600 мс)
    if (millis() - lastResetClickTime <= 600) {
      resetClicks++;
    } else {
      resetClicks = 1; 
    }
    lastResetClickTime = millis();

    // Обычный аварийный сброс (1 клик)
    if (currentState != WAIT_DENSITY && currentState != RETURN_TO_HOME) {  
      executeEmergencyReset();
    } 
    // Отмена ручного возврата планки
    else if (currentState == RETURN_TO_HOME) {
      Serial.println(F("--- HOMING ABORTED ---"));
      motorOff();
      horn.play(1, 600);
      beacon.setMode(0);
      currentState = WAIT_DENSITY;
    }

    // Запуск ручного возврата планки (5 кликов подряд)
    if (resetClicks == 5 && currentState == WAIT_DENSITY) {
      Serial.println(F("--- HOMING MODE INITIATED ---"));
      resetClicks = 0;
      motorOn();
      horn.play(1, 800);      
      beacon.setMode(1);
      stateTimer = millis();
      currentState = RETURN_TO_HOME;
    }
  }

  // =================================================================================
  // КОНЕЧНЫЙ АВТОМАТ (STATE MACHINE) - Сердце логики пресса
  // Только одно условие выполняется в любой момент времени
  // =================================================================================
  switch (currentState) {
    
    // СТАТУС 0: Спокойно набираем сено, ждем сжатия
    case WAIT_DENSITY:
      // Если концевик плотности зажат дольше, чем настройка защиты от кочек (plotnostLozh)
      if (densSensor.isHeldFor(cfg.plotnostLozh)) {
        horn.play(3, 300); // 3 коротких гудка трактористу "Стой!"
        beacon.setMode(1); // Включаем маячок
        stateTimer = millis(); // Засекаем время
        currentState = WAIT_TRACTOR; // Переходим на следующий шаг
      }
      break;

    // СТАТУС 1: Ждем, пока тракторист нажмет на тормоз
    case WAIT_TRACTOR:
      if (millis() - stateTimer >= cfg.timeForStopTraktor) {
        motorOn(); // Запускаем мотор обвязки (Шпагат или Сетка)
        stateTimer = millis(); 
        currentState = WAIT_START_SENSOR; // Переходим на следующий шаг
      }
      break;

    // СТАТУС 2: Мотор запущен, ждем физического подтверждения, что механизм поехал
    case WAIT_START_SENSOR:
      if (startSensor.isPressed()) {
        horn.play(1, 800); // Один длинный гудок "Обвязка пошла"            
        stateTimer = millis(); 
        currentState = MOTOR_RUNNING_TIMER;
      } 
      // Если мотор работает слишком долго, а механизм не тронулся - заклинило! Авария!
      else if (millis() - stateTimer >= cfg.timeoutMotorMax) {
        motorOff(); 
        beacon.setMode(2); // Быстро мигаем маячком (Авария)           
        stateTimer = millis(); 
        currentState = ERROR_STATE;    
      }
      break;

    // СТАТУС 3: Обвязка идет. Опираемся не на концевики, а на таймер работы мотора
    case MOTOR_RUNNING_TIMER:
      {
        // Выбираем, сколько секунд крутить мотор в зависимости от тумблера
        uint32_t currentRunTime = isNetModeActive() ? cfg.timerNetRun : cfg.timerTwineRun;
        if (millis() - stateTimer >= currentRunTime) {
          motorOff(); // Время вышло, глушим мотор
          stateTimer = millis(); 
          currentState = WAIT_END_SENSOR; // Переходим к ожиданию отрезки
        }
      }
      break;

    // СТАТУС 4: Обвязка намотана, ждем отрезания нити/сетки
    case WAIT_END_SENSOR:
      if (endSensor.isPressed()) {
        horn.play(2, 400); // 2 гудка "Готово, открывай!"
        doorWasOpened = false; 
        currentState = WAIT_DOOR;
      }
      // Если отрезка не произошла за отведенное время - Авария!
      else if (millis() - stateTimer >= cfg.timeoutEndSensor) {
        beacon.setMode(2);
        stateTimer = millis();
        currentState = ERROR_STATE;
      }
      break;

    // СТАТУС 5: Тракторист открывает заднюю камеру (Дверь) и выкидывает рулон
    case WAIT_DOOR:
      // Шаг А: фиксируем, что дверь была физически открыта
      if (doorSensor.isPressed() && !doorWasOpened) {
        doorWasOpened = true;
      }
      // Шаг Б: фиксируем, что дверь снова закрыта
      else if (!doorSensor.isPressed() && doorWasOpened) {
        // Успех! Рулон готов и выгружен. Плюсуем счетчики.
        txData.totalBales++; 
        txData.sessionBales++; // Эти данные улетят на пульт при следующем опросе RS485
        
        doorWasOpened = false; 
        beacon.setMode(0); // Гасим маяк            
        currentState = WAIT_DENSITY; // Цикл завершен, возвращаемся в начало!
      }
      break;

    // СТАТУС 6: Аварийное состояние. Пресс заблокирован до ручного сброса
    case ERROR_STATE:
      // Периодически пищим, чтобы привлечь внимание
      if (millis() - stateTimer >= 4000) { 
        horn.play(2, 200); 
        stateTimer = millis(); 
      }
      break;

    // СТАТУС 8: Принудительный возврат обвязывающей планки в стартовое положение (5 кликов)
    case RETURN_TO_HOME:
      if (endSensor.isPressed()) {
        motorOff();
        horn.play(2, 400); 
        beacon.setMode(0);
        currentState = WAIT_DENSITY;
      }
      else if (millis() - stateTimer >= cfg.timeoutReturnHome) {
        motorOff();
        beacon.setMode(2);
        stateTimer = millis();
        currentState = ERROR_STATE; // Защита от сжигания мотора при заклинивании
      }
      break;

    // СТАТУС 7: Режим тестирования концевиков (для отладки проводки)
    case TEST_MODE:
      // Автовыход через 60 секунд
      if (millis() - testModeStartTime >= 60000) {
        horn.play(2, 400); 
        beacon.setMode(0);             
        doorWasOpened = doorSensor.isPressed(); 
        currentState = WAIT_DENSITY;
      } else {
        // Если ничего не пищит, проверяем датчики
        if (!horn.isBusy()) {
          // Если нажат любой из 4 датчиков - включаем гудок. Так можно прозванивать цепи в одного!
          if (densSensor.isPressed() || startSensor.isPressed() || endSensor.isPressed() || doorSensor.isPressed()) {
            digitalWrite(PIN_RELAY_SOUND, cfg.soundActiveHigh ? HIGH : LOW);
          } else {
            digitalWrite(PIN_RELAY_SOUND, cfg.soundActiveHigh ? LOW : HIGH);
          }
        }
      }
      break;
  }
}
