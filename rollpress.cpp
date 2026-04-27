/*
 * =====================================================================================
 * ПРОЕКТ: Система управления рулонным пресс-подборщиком (Блок Пресса / Slave)
 * ВЕРСИЯ: 1.0 (Release)
 * ОПИСАНИЕ: 
 * Ядро системы. Устанавливается на самом пресс-подборщике (Arduino Nano/Uno).
 * Управляет силовыми реле, опрашивает оптические/индуктивные датчики (концевики),
 * реализует автомат состояний (State Machine) обвязки тюка.
 * Хранит настройки и счетчик Тотала на внешнем I2C EEPROM модуле (AT24Cxx).
 * Общается с пультом в кабине через RS485. При обрыве связи способен работать автономно.
 * =====================================================================================
 */

#include <SoftwareSerial.h>
#include <Wire.h>     // Библиотека для работы с шиной I2C (для внешнего EEPROM)
#include <avr/wdt.h>  // Библиотека Сторожевого таймера (Hardware Watchdog)

// =================================================================================
// РАСПИНОВКА (PINS) - БЛОК ПРЕССА
// =================================================================================
#define PIN_DENSITY        2    // Вход: Датчик набора нужной плотности (оптрон)
#define PIN_S_START        3    // Вход: Датчик захвата (Начало обвязки)
#define PIN_S_END          4    // Вход: Датчик отрезки ножом (Конец обвязки)
#define PIN_DOOR           5    // Вход: Датчик открытой двери камеры (оптрон)
#define PIN_RELAY_TWINE    6    // Выход: Реле мотора обвязки ШПАГАТОМ
#define PIN_RELAY_SOUND    7    // Выход: Реле звукового сигнала (гудок)
#define PIN_RESET          8    // Вход: Резервная локальная кнопка на самом блоке
#define PIN_RELAY_LIGHT    9    // Выход: Реле проблескового маячка
#define PIN_RELAY_NET      10   // Выход: Реле мотора обвязки СЕТКОЙ
#define PIN_SWITCH_NET     11   // Вход: Локальный тумблер (Сетка/Шпагат) на корпусе

#define PIN_RS485_RX       A0   // RX-пин для MAX485
#define PIN_RS485_TX       A1   // TX-пин для MAX485
#define PIN_RS485_EN       A2   // Управление направлением (DE/RE) MAX485

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// =================================================================================
// ВНЕШНИЙ EEPROM (I2C AT24Cxx) - КАРТА ПАМЯТИ
// =================================================================================
#define EEPROM_ADDR 0x50       // Стандартный I2C адрес чипов памяти AT24Cxx
#define EEPROM_MAGIC_BYTE 0x42 // Маркер проверки ("Магическое число" первичной разметки)
#define ADDR_MAGIC        0    // 1 байт: Адрес маркера
#define ADDR_TOTAL_BALES  1    // 4 байта: Адрес счетчика Тотала (uint32_t)
#define ADDR_T_DENS       5    // 1 байт: Настройка "Плотность (Анти-кочка)"
#define ADDR_T_STOP       6    // 1 байт: Настройка "Время на остановку трактора"
#define ADDR_T_NET        7    // 1 байт: Настройка "Время вращения мотора (Сетка)"
#define ADDR_T_TWINE      8    // 1 байт: Настройка "Время вращения мотора (Шпагат)"

// Переменные, загружаемые из памяти
uint32_t totalBales = 0;
uint8_t t_Dens = 1;   
uint8_t t_Stop = 4;
uint8_t t_Net = 2;
uint8_t t_Twine = 3;

// --- ЖЕСТКИЕ НАСТРОЙКИ (Конфигурация железа) ---
struct Config {
  const uint32_t timeoutMotorMax = 10000;   // Аварийное отключение сгоревшего мотора (10 сек)
  const uint32_t timeoutEndSensor = 15000;  // Таймаут ожидания падения ножа (15 сек)
  const uint32_t timeoutReturnHome = 10000; // Таймаут возврата механизма в исходное
  const uint32_t debounce = 50;             // Антидребезг датчиков (мс)
  
  // Настройки логики (High/Low) для разных модулей реле (Инверсия)
  const bool motorActiveHigh = true;  
  const bool soundActiveHigh = true; // Реле гудка срабатывает на LOW
  const bool lightActiveHigh = true;  
};
Config cfg;

// =================================================================================
// СТРУКТУРЫ ДАННЫХ ДЛЯ ОБМЕНА ПО RS485
// =================================================================================
struct MasterData { 
  bool doReset; 
  bool isManualMode; 
  bool isNetMode; 
  bool saveSettings; // Флаг-приказ: "Перезапиши настройки в EEPROM!"
  uint8_t timeoutDens;
  uint8_t timeoutStop;
  uint8_t timeoutNet;
  uint8_t timeoutTwine;
};
MasterData rxData;

struct SlaveData {
  uint8_t currentState;  
  uint16_t sessionBales; 
  uint32_t totalBales;   
  uint8_t t_Dens;        
  uint8_t t_Stop;
  uint8_t t_Net;
  uint8_t t_Twine;
};
SlaveData txData; 

// Переменные Сторожа связи (Если от пульта нет пакетов - переходим в автономию)
unsigned long lastMasterPacketTime = 0;
bool isRemoteConnected = false;

// =================================================================================
// ФУНКЦИИ РАБОТЫ С ВНЕШНИМ EEPROM ЧЕРЕЗ I2C
// (Написаны вручную для независимости от сторонних библиотек)
// =================================================================================
void writeEEPROM_Byte(uint16_t mem_addr, uint8_t data) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(mem_addr >> 8));   // Старший байт адреса памяти
  Wire.write((int)(mem_addr & 0xFF)); // Младший байт адреса памяти
  Wire.write(data);                   // Сами данные
  Wire.endTransmission();
  delay(5); // Аппаратная пауза, необходимая чипу для завершения прожига ячейки
}

uint8_t readEEPROM_Byte(uint16_t mem_addr) {
  uint8_t data = 0xFF;
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(mem_addr >> 8));
  Wire.write((int)(mem_addr & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 1);
  if (Wire.available()) data = Wire.read();
  return data;
}

// Запись 32-битного числа (uint32_t) по байтам
void writeEEPROM_Long(uint16_t mem_addr, uint32_t data) {
  writeEEPROM_Byte(mem_addr, (data & 0xFF));
  writeEEPROM_Byte(mem_addr + 1, ((data >> 8) & 0xFF));
  writeEEPROM_Byte(mem_addr + 2, ((data >> 16) & 0xFF));
  writeEEPROM_Byte(mem_addr + 3, ((data >> 24) & 0xFF));
}

// Чтение 32-битного числа (uint32_t) по байтам
uint32_t readEEPROM_Long(uint16_t mem_addr) {
  uint32_t data = 0;
  data = readEEPROM_Byte(mem_addr);
  data |= ((uint32_t)readEEPROM_Byte(mem_addr + 1) << 8);
  data |= ((uint32_t)readEEPROM_Byte(mem_addr + 2) << 16);
  data |= ((uint32_t)readEEPROM_Byte(mem_addr + 3) << 24);
  return data;
}

// =================================================================================
// КЛАССЫ ПЕРИФЕРИИ (Датчики, Гудок, Маячок)
// =================================================================================

// Класс для чтения датчиков с программным фильтром дребезга
class Sensor {
  private:
    uint8_t pin; 
    bool invertLogic; // true, если датчик замыкает на GND
    unsigned long lastChange; 
    unsigned long stateChangeTime;
    bool state; 
    bool lastReading; 
    bool stateChanged;
  public:
    Sensor(uint8_t p, bool inv = true) : pin(p), invertLogic(inv), lastChange(0), stateChangeTime(0), state(false), lastReading(false), stateChanged(false) {}
    
    void begin() { pinMode(pin, invertLogic ? INPUT_PULLUP : INPUT); }
    
    void update() {
      stateChanged = false; 
      bool reading = digitalRead(pin);
      if (invertLogic) reading = !reading; // Инверсия сигнала NPN-датчиков
      
      if (reading != lastReading) lastChange = millis();
      
      if ((millis() - lastChange) >= cfg.debounce) {
        if (state != reading) { 
          state = reading; 
          stateChanged = true; 
          stateChangeTime = millis(); // Фиксация времени, когда состояние точно изменилось
        }
      }
      lastReading = reading;
    }
    
    bool isPressed() { return state; } 
    bool justPressed() { return (state == true && stateChanged == true); } 
    bool isHeldFor(uint32_t time) { return state && ((millis() - stateChangeTime) >= time); } 
};

// Класс неблокирующего Гудка (позволяет пищать "азбукой морзе" без функции delay)
class Signaler {
  private:
    uint8_t pin; 
    int beepsLeft = 0; 
    unsigned long lastToggle = 0; 
    bool isRelayOn = false; 
    uint32_t duration;
    void turnOn()  { digitalWrite(pin, cfg.soundActiveHigh ? HIGH : LOW); }
    void turnOff() { digitalWrite(pin, cfg.soundActiveHigh ? LOW : HIGH); }
  public:
    Signaler(uint8_t p) : pin(p) {}
    void begin() { pinMode(pin, OUTPUT); turnOff(); }
    
    // Задать количество гудков и их длительность
    void play(int count, uint32_t dur = 300) { 
      beepsLeft = count * 2; 
      duration = dur; 
      isRelayOn = true; 
      turnOn(); 
      lastToggle = millis(); 
      beepsLeft--; 
    }
    
    void update() {
      if (beepsLeft > 0 && (millis() - lastToggle >= duration)) {
        isRelayOn = !isRelayOn; 
        if (isRelayOn) turnOn(); else turnOff();
        lastToggle = millis(); 
        beepsLeft--;
      }
    }
    bool isBusy() { return beepsLeft > 0; } 
};

// Класс управления световым маячком (Постоянный свет или разные режимы мигания)
class LightController {
  private:
    uint8_t pin; 
    uint8_t currentMode; 
    unsigned long lastToggle; 
    bool state;
    void turnOn()  { digitalWrite(pin, cfg.lightActiveHigh ? HIGH : LOW); }
    void turnOff() { digitalWrite(pin, cfg.lightActiveHigh ? LOW : HIGH); }
  public:
    LightController(uint8_t p) : pin(p), currentMode(0), lastToggle(0), state(false) {}
    void begin() { pinMode(pin, OUTPUT); turnOff(); }
    
    // 0 = Выкл, 1 = Горит постоянно, 2 = Быстро мигает (Авария), 3 = Медленно мигает (Тест)
    void setMode(uint8_t mode) { 
      currentMode = mode; 
      if (mode == 0) { state = false; turnOff(); } 
      else if (mode == 1) { state = true; turnOn(); } 
    }
    
    void update() {
      if (currentMode == 2) { 
        if (millis() - lastToggle >= 300) { state = !state; if (state) turnOn(); else turnOff(); lastToggle = millis(); } 
      } 
      else if (currentMode == 3) { 
        if (millis() - lastToggle >= 1000) { state = !state; if (state) turnOn(); else turnOff(); lastToggle = millis(); } 
      }
    }
};

// Инициализация объектов периферии
Sensor densSensor(PIN_DENSITY, true); 
Sensor startSensor(PIN_S_START, true);
Sensor endSensor(PIN_S_END, true);
Sensor doorSensor(PIN_DOOR, false); // У датчика двери логика может отличаться
Sensor resetBtn(PIN_RESET, true); 

Signaler horn(PIN_RELAY_SOUND);
LightController beacon(PIN_RELAY_LIGHT);

// =================================================================================
// АВТОМАТ СОСТОЯНИЙ (STATE MACHINE) ПРЕССА
// =================================================================================
enum BalerState { 
  WAIT_DENSITY,        // 0: Ожидание наполнения камеры (Набор массы)
  WAIT_TRACTOR,        // 1: Камера полная, ожидание остановки трактора
  WAIT_START_SENSOR,   // 2: Мотор запущен, ожидаем датчик захвата (Старт)
  MOTOR_RUNNING_TIMER, // 3: Вращение мотора по заданному таймеру (Обвязка)
  WAIT_END_SENSOR,     // 4: Ждем падения ножа или отсечки (Датчик Конец)
  WAIT_DOOR,           // 5: Ожидаем открытия и закрытия задней двери (Выброс)
  ERROR_STATE,         // 6: Глобальная авария (сбой датчиков/мотора)
  TEST_MODE,           // 7: Режим тестирования концевиков со звуком
  RETURN_TO_HOME       // 8: Режим принудительного возврата механизма
};
BalerState currentState = WAIT_DENSITY; 

unsigned long stateTimer = 0;        
unsigned long testModeStartTime = 0; 
bool doorWasOpened = false;          
uint8_t resetClicks = 0;             
unsigned long lastResetClickTime = 0;
uint16_t sessionBales = 0;

// Умный выбор материала: Если пульт на связи - слушаем пульт. Иначе - локальный тумблер.
bool getNetMode() { 
  if (isRemoteConnected) return rxData.isNetMode; 
  else return !digitalRead(PIN_SWITCH_NET); 
}

void motorOn() { 
  if (getNetMode()) digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? HIGH : LOW);
  else digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? HIGH : LOW);
}

void motorOff() { 
  digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? LOW : HIGH);
  digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? LOW : HIGH);
}

// Экстренный сброс (возврат в начальное состояние)
void executeEmergencyReset() {
  if (currentState != WAIT_DENSITY && currentState != TEST_MODE) {  
    motorOff(); 
    horn.play(1, 600); 
    beacon.setMode(0); 
    doorWasOpened = doorSensor.isPressed(); 
    currentState = WAIT_DENSITY;           
  }
}

// Отправка ответного пакета на Пульт
void sendRS485Reply() {
  digitalWrite(PIN_RS485_EN, HIGH); 
  delay(2);
  
  // Обновляем структуру актуальными данными
  txData.currentState = currentState; 
  txData.sessionBales = sessionBales;
  txData.totalBales = totalBales;
  txData.t_Dens = t_Dens;
  txData.t_Stop = t_Stop;
  txData.t_Net = t_Net;
  txData.t_Twine = t_Twine;

  rs485.write(0xAA); // Стартовый байт
  uint8_t crc = 0; 
  uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(SlaveData); i++) { 
    rs485.write(ptr[i]); 
    crc ^= ptr[i]; 
  }
  rs485.write(crc); 
  rs485.flush(); 
  digitalWrite(PIN_RS485_EN, LOW); 
}

// Прослушивание шины RS485 на наличие команд
void listenRS485() {
  if (rs485.available() > 0) { 
    if (rs485.read() == 0xBB) { 
      // Динамический таймаут ожидания длинного пакета (до 30 мс)
      unsigned long pStart = millis();
      while (rs485.available() < sizeof(MasterData) + 1) {
        if (millis() - pStart > 30) break; 
      }

      // Если пакет пришел целиком
      if (rs485.available() >= sizeof(MasterData) + 1) { 
        uint8_t crc = 0; 
        uint8_t* ptr = (uint8_t*)&rxData;
        for (uint16_t i = 0; i < sizeof(MasterData); i++) { 
          ptr[i] = rs485.read(); 
          crc ^= ptr[i]; 
        }
        
        // Проверка целостности
        if (crc == rs485.read()) {
          lastMasterPacketTime = millis();
          isRemoteConnected = true; // Сбрасываем сторож обрыва связи

          // Если пульт передал новые настройки (пользователь нажал Сохранить)
          if (rxData.saveSettings) {
            t_Dens = rxData.timeoutDens;
            t_Stop = rxData.timeoutStop;
            t_Net = rxData.timeoutNet;
            t_Twine = rxData.timeoutTwine;
            // СОХРАНЯЕМ ВО ВНЕШНЮЮ ПАМЯТЬ I2C
            writeEEPROM_Byte(ADDR_T_DENS, t_Dens);
            writeEEPROM_Byte(ADDR_T_STOP, t_Stop);
            writeEEPROM_Byte(ADDR_T_NET, t_Net);
            writeEEPROM_Byte(ADDR_T_TWINE, t_Twine);
          }

          if (rxData.doReset) executeEmergencyReset(); 
          
          sendRS485Reply(); // Сразу отправляем ответ
        }
      }
    }
  }
}

// =================================================================================
// СТАРТОВАЯ ИНИЦИАЛИЗАЦИЯ ПРЕССА
// =================================================================================
void setup() {
  Wire.begin(); // Инициализация шины I2C для связи с EEPROM
  
  // Проверяем, была ли память ранее отформатирована нашей программой
  uint8_t magic = readEEPROM_Byte(ADDR_MAGIC);
  if (magic == EEPROM_MAGIC_BYTE) {
    // Память не пустая, безопасно читаем все данные
    totalBales = readEEPROM_Long(ADDR_TOTAL_BALES);
    t_Dens = readEEPROM_Byte(ADDR_T_DENS);
    t_Stop = readEEPROM_Byte(ADDR_T_STOP);
    t_Net = readEEPROM_Byte(ADDR_T_NET);
    t_Twine = readEEPROM_Byte(ADDR_T_TWINE);
  } else {
    // Первый запуск на новом чипе EEPROM: записываем начальные настройки и маркер
    writeEEPROM_Long(ADDR_TOTAL_BALES, totalBales);
    writeEEPROM_Byte(ADDR_T_DENS, t_Dens);
    writeEEPROM_Byte(ADDR_T_STOP, t_Stop);
    writeEEPROM_Byte(ADDR_T_NET, t_Net);
    writeEEPROM_Byte(ADDR_T_TWINE, t_Twine);
    writeEEPROM_Byte(ADDR_MAGIC, EEPROM_MAGIC_BYTE);
  }

  // Настройка периферии
  pinMode(PIN_RS485_EN, OUTPUT); digitalWrite(PIN_RS485_EN, LOW); rs485.begin(9600); 
  densSensor.begin(); startSensor.begin(); endSensor.begin(); doorSensor.begin(); resetBtn.begin();
  pinMode(PIN_SWITCH_NET, INPUT_PULLUP); 
  horn.begin(); beacon.begin();
  pinMode(PIN_RELAY_TWINE, OUTPUT); pinMode(PIN_RELAY_NET, OUTPUT);
  
  motorOff(); beacon.setMode(0); 

  // Включаем аппаратный сторожевой таймер. При зависании кода плата уйдет в Reset
  wdt_enable(WDTO_2S); 
}

// =================================================================================
// ОСНОВНОЙ ЦИКЛ БЛОКА ПРЕССА
// =================================================================================
void loop() {
  wdt_reset(); // Обнуление сторожевого таймера

  // Опрос физики
  densSensor.update(); startSensor.update(); endSensor.update(); doorSensor.update(); resetBtn.update();
  horn.update(); beacon.update(); 
  listenRS485();

  // Сторожевой таймер связи (Watchdog RS485). 
  // Если от пульта 2 сек нет вестей - переходим на локальное управление.
  if (isRemoteConnected && (millis() - lastMasterPacketTime > 2000)) {
    isRemoteConnected = false;
  }

  // --- ЛОГИКА РЕЗЕРВНОЙ ЛОКАЛЬНОЙ КНОПКИ (На ящике пресса) ---
  bool isResetJustPressed = resetBtn.justPressed();

  // Удержание 10 сек: Вход в режим Теста датчиков (Озвучка срабатываний)
  if (resetBtn.isHeldFor(10000) && currentState != TEST_MODE) {
    motorOff(); horn.play(1, 100); beacon.setMode(3); currentState = TEST_MODE; testModeStartTime = millis(); resetClicks = 0; 
  }

  if (isResetJustPressed && currentState != TEST_MODE) {
    if (millis() - lastResetClickTime <= 600) resetClicks++; else resetClicks = 1; 
    lastResetClickTime = millis();

    // Экстренный сброс алгоритма по 1 нажатию
    if (currentState != WAIT_DENSITY && currentState != RETURN_TO_HOME) { executeEmergencyReset(); } 
    else if (currentState == RETURN_TO_HOME) { motorOff(); horn.play(1, 600); beacon.setMode(0); currentState = WAIT_DENSITY; }

    // Аварийный принудительный возврат планки по 5 быстрым кликам
    if (resetClicks == 5 && currentState == WAIT_DENSITY) {
      resetClicks = 0; motorOn(); horn.play(1, 800); beacon.setMode(1); stateTimer = millis(); currentState = RETURN_TO_HOME;
    }
  }

  // =================================================================================
  // СЕРДЦЕ СИСТЕМЫ: АВТОМАТ СОСТОЯНИЙ (STATE MACHINE)
  // =================================================================================
  switch (currentState) {
    
    // ШАГ 0: Ждем наполнения камеры сеном
    case WAIT_DENSITY: {
      // Исключение: В ручном режиме ждем ровно 1 сек (чтобы тракторист мог замерить и откалибровать время)
      // В Авторежиме берем время из настройки (умножаем на 1000, так как нужны миллисекунды)
      uint32_t delayDens = (isRemoteConnected && rxData.isManualMode) ? 1000UL : (t_Dens * 1000UL);
      
      // Если датчик плотности был непрерывно нажат заданное время (защита от кочек пройдена)
      if (densSensor.isHeldFor(delayDens)) { 
        horn.play(3, 300); // 3 гудка - сигнал "Стоп Трактор!"
        beacon.setMode(1); // Включаем маячок
        stateTimer = millis(); 
        currentState = WAIT_TRACTOR; 
      } 
      break;
    }
    
    // ШАГ 1: Даем трактористу время остановиться перед подачей сетки
    case WAIT_TRACTOR:
      if (isRemoteConnected && rxData.isManualMode) {
        // Если ручной режим - мы сами мотор НЕ включаем. Ждем, пока тракторист нажмет аппаратную кнопку
        // которая аппаратно включит реле мотора. Как только мотор дернет планку, сработает датчик Старта.
        if (startSensor.isPressed()) { horn.play(1, 800); stateTimer = millis(); currentState = MOTOR_RUNNING_TIMER; }
      } else {
        // Авторежим: ждем установленное время t_Stop и программно запускаем мотор
        if (millis() - stateTimer >= (t_Stop * 1000UL)) { motorOn(); stateTimer = millis(); currentState = WAIT_START_SENSOR; } 
      }
      break;

    // ШАГ 2: Ждем механического подтверждения начала обвязки
    case WAIT_START_SENSOR:
      if (startSensor.isPressed()) { 
        horn.play(1, 800); stateTimer = millis(); currentState = MOTOR_RUNNING_TIMER; 
      } 
      // Если мотор крутится, а датчик так и не сработал (порвалась цепь, сгорел мотор) - уходим в Аварию
      else if (millis() - stateTimer >= cfg.timeoutMotorMax) { 
        motorOff(); beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; 
      } 
      break;
    
    // ШАГ 3: Обвязка (Отматываем нужную длину сетки или шпагата)
    case MOTOR_RUNNING_TIMER:
      // Крутим мотор согласно выбранному материалу (Сетка или Шпагат)
      if (millis() - stateTimer >= (getNetMode() ? (t_Net * 1000UL) : (t_Twine * 1000UL))) { 
        motorOff(); stateTimer = millis(); currentState = WAIT_END_SENSOR; 
      } 
      break;
    
    // ШАГ 4: Ждем отсечки (падения ножа)
    case WAIT_END_SENSOR:
      if (endSensor.isPressed()) { 
        horn.play(2, 400); // 2 гудка - сигнал "Выбрасывай тюк!"
        doorWasOpened = false; 
        currentState = WAIT_DOOR; 
      }
      // Если нож не упал слишком долго - Авария (затупился нож, застряла нить)
      else if (millis() - stateTimer >= cfg.timeoutEndSensor) { 
        beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; 
      } 
      break;
    
    // ШАГ 5: Ожидание открытия и закрытия двери камеры (Выгрузка)
    case WAIT_DOOR:
      if (doorSensor.isPressed() && !doorWasOpened) {
        doorWasOpened = true; // Зафиксировали факт открытия
      }
      else if (!doorSensor.isPressed() && doorWasOpened) { 
        // Дверь закрылась! Цикл успешно завершен.
        totalBales++; 
        sessionBales++; 
        // НАДЕЖНО СОХРАНЯЕМ ТОТАЛ ВО ВНЕШНЮЮ I2C ПАМЯТЬ
        writeEEPROM_Long(ADDR_TOTAL_BALES, totalBales); 
        
        doorWasOpened = false; 
        beacon.setMode(0); // Выключаем маяк
        currentState = WAIT_DENSITY; // Возврат в начало
      } 
      break;
    
    // ШАГ 6: Режим АВАРИИ
    case ERROR_STATE:
      // Бесконечный цикл писков, вывести из него может только кнопка Сброса
      if (millis() - stateTimer >= 4000) { horn.play(2, 200); stateTimer = millis(); } 
      break;
    
    // ШАГ 8: Принудительный возврат механизма в исходное положение
    case RETURN_TO_HOME:
      if (endSensor.isPressed()) { motorOff(); horn.play(2, 400); beacon.setMode(0); currentState = WAIT_DENSITY; }
      else if (millis() - stateTimer >= cfg.timeoutReturnHome) { motorOff(); beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; } 
      break;
    
    // ШАГ 7: Режим ТЕСТА железа (очень помогает при настройке датчиков на прессе)
    case TEST_MODE:
      // Автовыход через 60 секунд
      if (millis() - testModeStartTime >= 60000) { 
        horn.play(2, 400); beacon.setMode(0); doorWasOpened = doorSensor.isPressed(); currentState = WAIT_DENSITY; 
      } 
      else { 
        // Гудок напрямую дублирует нажатие ЛЮБОГО датчика
        if (!horn.isBusy()) { 
          digitalWrite(PIN_RELAY_SOUND, (densSensor.isPressed() || startSensor.isPressed() || endSensor.isPressed() || doorSensor.isPressed()) ? (cfg.soundActiveHigh ? HIGH : LOW) : (cfg.soundActiveHigh ? LOW : HIGH)); 
        } 
      } 
      break;
  }
}
