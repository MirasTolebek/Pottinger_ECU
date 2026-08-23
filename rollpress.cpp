/*
 * ПРОЕКТ: Система управления пресс-подборщиком (Блок Пресса / Slave)
 * ВЕРСИЯ: 3.2 "Неубиваемая + сброс через дверь + правильный порядок чекпоинта" 
 * ИЗМЕНЕНИЯ vs 3.1:
 *   1. MCUSR=0 / wdt_disable() ПЕРВЫМИ строчками setup().
 *   2. Очистка "хвоста" буфера RS485 при неполном/битом пакете от Мастера.
 *   3. Сброс цикла через концевик двери (по заводской схеме) - см. комментарий
 *      в loop() перед switch(currentState).
 *   4. НОВОЕ: в WAIT_DOOR порядок операций изменён - saveRecoveryStatus(0)
 *      теперь вызывается ПЕРВЫМ, до записи счётчиков тюков и звука. Раньше,
 *      если сброс платы (помеха от двигателя) случался ПОСЛЕ инкремента
 *      totalBales/sessionBales, но ДО saveRecoveryStatus(0) - в EEPROM
 *      оставался валидный старый чекпоинт status=2 ("тюк ещё не выгружен").
 *      При перезапуске система честно восстанавливала WAIT_DOOR и намертво
 *      игнорировала набор плотности, ожидая выгрузки уже выброшенного тюка -
 *      именно этот баг проявился в поле и был вычислен по длине сигнала
 *      recStatus==2 (5 гудков по 200мс = 2 сек) при перезапуске.
 *
 *   ПРИМЕЧАНИЕ: идея автокоррекции по "зависшему" концевику ножа была
 *   рассмотрена и ОТКЛОНЕНА - датчики плотности и ножа не позволяют надёжно
 *   отличить "тюк только что готов" от "тюк давно готов, ещё не выгружен,
 *   едем с ним в камере к месту разгрузки". Ложное срабатывание в этой
 *   развилке рискует запустить повторную обвязку поверх готового тюка.
 */

#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/wdt.h> // Библиотека аппаратного Сторожевого пса

// ================= НАСТРОЙКИ ПИНОВ =================
#define PIN_DENSITY        2    // Датчик плотности
#define PIN_S_START        3    // Датчик начала обвязки (№1)
#define PIN_S_END          4    // Датчик конца обвязки / парковки (№2)
#define PIN_DOOR           5    // Датчик закрытия двери
#define PIN_RELAY_TWINE    6    // Реле мотора (Шпагат)
#define PIN_RELAY_SOUND    7    // Реле ЗУММЕРА (Сирена)
#define PIN_RESET          8    // Кнопка Сброс/Готово (Аварийная)
#define PIN_RELAY_LIGHT    9    // Реле проблескового маячка
#define PIN_RELAY_NET      10   // Реле мотора (Сетка)
#define PIN_SWITCH_NET     11   // Тумблер выбора "Сетка / Шпагат"

#define PIN_RS485_RX       A0   
#define PIN_RS485_TX       A1   
#define PIN_RS485_EN       A2   

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// ================= EEPROM И ПАМЯТЬ =================
#define EEPROM_ADDR 0x50       // Адрес I2C модуля памяти
#define EEPROM_MAGIC_BYTE 0x45 

// Адреса памяти настроек
#define ADDR_MAGIC         0    
#define ADDR_TOTAL_BALES   1    
#define ADDR_SESSION_BALES 5   
#define ADDR_T_DENS        7    
#define ADDR_T_STOP        8    
#define ADDR_T_NET         9    
#define ADDR_T_TWINE       10   
#define ADDR_SOUND_MODE    11   
#define ADDR_T_MOTOR       12   

// --- АДРЕСА ДЛЯ УМНОГО ВОССТАНОВЛЕНИЯ (v3.0) ---
#define ADDR_BALE_STATUS   100  // Текущий статус тюка (0, 1 или 2)
#define ADDR_BALE_CHECKSUM 101  // Инверсная контрольная сумма (255 - статус)

// Переменные статистики и настроек
uint32_t totalBales = 0;
uint16_t sessionBales = 0; 
uint8_t t_Dens = 1;   
uint8_t t_Stop = 4;
uint8_t t_Net = 2;
uint8_t t_Twine = 3;
uint8_t t_Motor = 40; 
uint8_t soundMode = 2; 

// Аппаратные настройки (Инверсия реле)
struct Config {
  const uint32_t debounce = 150;            
  const bool motorActiveHigh = true;  
  const bool soundActiveHigh = true;  
  const bool lightActiveHigh = true;  
};
Config cfg;

// Структуры для RS485
struct MasterData { 
  bool doReset; bool isManualMode; bool isNetMode; bool saveSettings; bool resetSession;
  uint8_t timeoutDens; uint8_t timeoutStop; uint8_t timeoutNet; uint8_t timeoutTwine;
  uint8_t timeoutMotor; uint8_t soundMode; 
};
MasterData rxData;

struct SlaveData {
  uint8_t currentState; uint16_t sessionBales; uint32_t totalBales;   
  uint8_t t_Dens; uint8_t t_Stop; uint8_t t_Net; uint8_t t_Twine;
  uint8_t t_Motor; uint8_t soundMode; 
};
SlaveData txData; 

unsigned long lastMasterPacketTime = 0;
bool isRemoteConnected = false;

// ================= ФУНКЦИИ РАБОТЫ С ПАМЯТЬЮ (EEPROM) =================
void writeEEPROM_Byte(uint16_t mem_addr, uint8_t data) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(mem_addr >> 8)); Wire.write((int)(mem_addr & 0xFF)); Wire.write(data);                   
  Wire.endTransmission(); delay(5); 
}
uint8_t readEEPROM_Byte(uint16_t mem_addr) {
  uint8_t data = 0xFF; Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(mem_addr >> 8)); Wire.write((int)(mem_addr & 0xFF)); Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 1); if (Wire.available()) data = Wire.read(); return data;
}
void updateEEPROM_Byte(uint16_t mem_addr, uint8_t data) {
  if (readEEPROM_Byte(mem_addr) == data) return; // Защита от износа (пишет только если данные другие)
  writeEEPROM_Byte(mem_addr, data);
}
// Функции для Int и Long
void writeEEPROM_Int(uint16_t mem_addr, uint16_t data) {
  updateEEPROM_Byte(mem_addr, (data & 0xFF)); updateEEPROM_Byte(mem_addr + 1, ((data >> 8) & 0xFF));
}
uint16_t readEEPROM_Int(uint16_t mem_addr) {
  uint16_t data = readEEPROM_Byte(mem_addr); data |= ((uint16_t)readEEPROM_Byte(mem_addr + 1) << 8); return data;
}
void writeEEPROM_Long(uint16_t mem_addr, uint32_t data) {
  updateEEPROM_Byte(mem_addr, (data & 0xFF)); updateEEPROM_Byte(mem_addr + 1, ((data >> 8) & 0xFF));
  updateEEPROM_Byte(mem_addr + 2, ((data >> 16) & 0xFF)); updateEEPROM_Byte(mem_addr + 3, ((data >> 24) & 0xFF));
}
uint32_t readEEPROM_Long(uint16_t mem_addr) {
  uint32_t data = readEEPROM_Byte(mem_addr); data |= ((uint32_t)readEEPROM_Byte(mem_addr + 1) << 8);
  data |= ((uint32_t)readEEPROM_Byte(mem_addr + 2) << 16); data |= ((uint32_t)readEEPROM_Byte(mem_addr + 3) << 24); return data;
}

// === УМНОЕ СОХРАНЕНИЕ СТАТУСА ТЮКА (Защита от сбоев питания) ===
void saveRecoveryStatus(uint8_t status) {
  updateEEPROM_Byte(ADDR_BALE_STATUS, status);
  updateEEPROM_Byte(ADDR_BALE_CHECKSUM, 255 - status); // Инверсная контрольная сумма
}

// ================= КЛАССЫ ОБОРУДОВАНИЯ =================
class Sensor {
  private:
    uint8_t pin; bool invertLogic; unsigned long lastChange; unsigned long stateChangeTime;
    bool state; bool lastReading; bool stateChanged;
  public:
    Sensor(uint8_t p, bool inv = true) : pin(p), invertLogic(inv), lastChange(0), stateChangeTime(0), state(false), lastReading(false), stateChanged(false) {}
    void begin() { pinMode(pin, invertLogic ? INPUT_PULLUP : INPUT); }
    void update() {
      stateChanged = false; bool reading = digitalRead(pin);
      if (invertLogic) reading = !reading; 
      if (reading != lastReading) lastChange = millis();
      if ((millis() - lastChange) >= cfg.debounce) {
        if (state != reading) { state = reading; stateChanged = true; stateChangeTime = millis(); }
      }
      lastReading = reading;
    }
    bool isPressed() { return state; } 
    bool justPressed() { return (state == true && stateChanged == true); } 
    bool isHeldFor(uint32_t time) { return state && ((millis() - stateChangeTime) >= time); } 
};

class Signaler {
  private:
    uint8_t pin; int beepsLeft = 0; unsigned long lastToggle = 0; bool isRelayOn = false; uint32_t duration;
    void turnOn()  { digitalWrite(pin, cfg.soundActiveHigh ? HIGH : LOW); }
    void turnOff() { digitalWrite(pin, cfg.soundActiveHigh ? LOW : HIGH); }
  public:
    Signaler(uint8_t p) : pin(p) {}
    void begin() { pinMode(pin, OUTPUT); turnOff(); }
    void play(int count, uint32_t dur = 300) { 
      bool shouldPlay = true;
      if (isRemoteConnected && soundMode == 1) { shouldPlay = false; } // Если пульт подключен и звук переведен в пульт
      if (shouldPlay) { beepsLeft = count * 2; duration = dur; isRelayOn = true; turnOn(); lastToggle = millis(); beepsLeft--; }
    }
    void update() {
      if (beepsLeft > 0 && (millis() - lastToggle >= duration)) {
        isRelayOn = !isRelayOn; if (isRelayOn) turnOn(); else turnOff(); lastToggle = millis(); beepsLeft--;
      }
    }
    bool isBusy() { return beepsLeft > 0; } 
};

class LightController {
  private:
    uint8_t pin; uint8_t currentMode; unsigned long lastToggle; bool state;
    void turnOn()  { digitalWrite(pin, cfg.lightActiveHigh ? HIGH : LOW); }
    void turnOff() { digitalWrite(pin, cfg.lightActiveHigh ? LOW : HIGH); }
  public:
    LightController(uint8_t p) : pin(p), currentMode(0), lastToggle(0), state(false) {}
    void begin() { pinMode(pin, OUTPUT); turnOff(); }
    void setMode(uint8_t mode) { currentMode = mode; if (mode == 0) { state = false; turnOff(); } else if (mode == 1) { state = true; turnOn(); } }
    void update() {
      if (currentMode == 2) { if (millis() - lastToggle >= 300) { state = !state; if (state) turnOn(); else turnOff(); lastToggle = millis(); } } 
      else if (currentMode == 3) { if (millis() - lastToggle >= 1000) { state = !state; if (state) turnOn(); else turnOff(); lastToggle = millis(); } }
    }
};

Sensor densSensor(PIN_DENSITY, true); 
Sensor startSensor(PIN_S_START, true);
Sensor endSensor(PIN_S_END, true); 
Sensor doorSensor(PIN_DOOR, false); 
Sensor resetBtn(PIN_RESET, true); 

Signaler horn(PIN_RELAY_SOUND); 
LightController beacon(PIN_RELAY_LIGHT);

// Конечный автомат (Все состояния пресса)
enum BalerState { WAIT_DENSITY, WAIT_TRACTOR, WAIT_START_SENSOR, MOTOR_RUNNING_TIMER, WAIT_END_SENSOR, WAIT_DOOR, ERROR_STATE, TEST_MODE, RETURN_TO_HOME };
BalerState currentState = WAIT_DENSITY; 

unsigned long stateTimer = 0;        
unsigned long testModeStartTime = 0; 
bool doorWasOpened = false;          
uint8_t resetClicks = 0;             
unsigned long lastResetClickTime = 0;

bool getNetMode() { if (isRemoteConnected) return rxData.isNetMode; else return !digitalRead(PIN_SWITCH_NET); }

void motorOn() { 
  if (getNetMode()) digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? HIGH : LOW);
  else digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? HIGH : LOW);
}
void motorOff() { digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? LOW : HIGH); digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? LOW : HIGH); }

void executeEmergencyReset() {
  if (currentState != WAIT_DENSITY && currentState != TEST_MODE) {  
    motorOff(); horn.play(1, 600); beacon.setMode(0); doorWasOpened = doorSensor.isPressed(); 
    resetClicks = 0; currentState = WAIT_DENSITY; 
    saveRecoveryStatus(0); // Сброс контрольной точки
  }
}

// ================= РАБОТА С ПУЛЬТОМ (RS485) =================
void sendRS485Reply() {
  digitalWrite(PIN_RS485_EN, HIGH); delay(2);
  txData.currentState = currentState; txData.sessionBales = sessionBales; txData.totalBales = totalBales;
  txData.t_Dens = t_Dens; txData.t_Stop = t_Stop; txData.t_Net = t_Net; txData.t_Twine = t_Twine;
  txData.t_Motor = t_Motor; txData.soundMode = soundMode; 
  rs485.write(0xAA); 
  uint8_t crc = 0; uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(SlaveData); i++) { rs485.write(ptr[i]); crc ^= ptr[i]; }
  rs485.write(crc); rs485.flush(); digitalWrite(PIN_RS485_EN, LOW); 
}

void listenRS485() {
  if (rs485.available() > 0) { 
    if (rs485.read() == 0xBB) { 
      unsigned long pStart = millis();
      while (rs485.available() < sizeof(MasterData) + 1) { if (millis() - pStart > 30) break; }
      if (rs485.available() >= sizeof(MasterData) + 1) { 
        uint8_t crc = 0; uint8_t* ptr = (uint8_t*)&rxData;
        for (uint16_t i = 0; i < sizeof(MasterData); i++) { ptr[i] = rs485.read(); crc ^= ptr[i]; }
        if (crc == rs485.read()) {
          lastMasterPacketTime = millis(); isRemoteConnected = true; 
          if (rxData.resetSession) { sessionBales = 0; writeEEPROM_Int(ADDR_SESSION_BALES, sessionBales); }
          if (rxData.saveSettings) {
            t_Dens = rxData.timeoutDens; t_Stop = rxData.timeoutStop; t_Net = rxData.timeoutNet;
            t_Twine = rxData.timeoutTwine; t_Motor = rxData.timeoutMotor; soundMode = rxData.soundMode; 
            updateEEPROM_Byte(ADDR_T_DENS, t_Dens); updateEEPROM_Byte(ADDR_T_STOP, t_Stop);
            updateEEPROM_Byte(ADDR_T_NET, t_Net); updateEEPROM_Byte(ADDR_T_TWINE, t_Twine);
            updateEEPROM_Byte(ADDR_T_MOTOR, t_Motor); updateEEPROM_Byte(ADDR_SOUND_MODE, soundMode); 
          }
          if (rxData.doReset) executeEmergencyReset(); 
          sendRS485Reply(); 
        }
      } else {
        // ИСПРАВЛЕНИЕ: если пакет не собрался целиком за отведённое время
        // (помеха/обрыв на линии) - чистим "хвост" буфера, иначе следующий
        // поиск заголовка 0xBB может зацепиться за случайный байт из этого
        // недополученного пакета и рассинхронизировать кадр.
        while (rs485.available()) rs485.read();
      }
    }
  }
}

// ================= SETUP (СТАРТ И ВОССТАНОВЛЕНИЕ) =================
void setup() {
  // === ОБЯЗАТЕЛЬНО ПЕРВЫМИ СТРОЧКАМИ! ===
  // Обнуляем MCUSR и глушим WDT сразу после старта. Если этого не сделать -
  // унаследованный после watchdog-сброса бит WDRF держит WDT включённым
  // со старым таймаутом ещё во время работы с EEPROM ниже (чтение настроек,
  // возможная первая запись, чтение recovery-статуса) - можно словить сброс
  // ПОСРЕДИ записи, что как раз и подрывает саму идею "неубиваемости".
  MCUSR = 0;
  wdt_disable();

  Wire.begin(); 
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true); // Защита шины I2C от зависаний
  #endif

  // 1. Чтение базовых настроек
  uint8_t magic = readEEPROM_Byte(ADDR_MAGIC);
  if (magic == EEPROM_MAGIC_BYTE) {
    totalBales = readEEPROM_Long(ADDR_TOTAL_BALES); sessionBales = readEEPROM_Int(ADDR_SESSION_BALES);
    t_Dens = readEEPROM_Byte(ADDR_T_DENS); t_Stop = readEEPROM_Byte(ADDR_T_STOP);
    t_Net = readEEPROM_Byte(ADDR_T_NET); t_Twine = readEEPROM_Byte(ADDR_T_TWINE);
    t_Motor = readEEPROM_Byte(ADDR_T_MOTOR); soundMode = readEEPROM_Byte(ADDR_SOUND_MODE);
    
    // Валидация
    if (soundMode > 2) soundMode = 2; 
    if (t_Motor != 0 && (t_Motor < 10 || t_Motor > 90)) t_Motor = 40;
    if (t_Dens < 1 || t_Dens > 20) t_Dens = 1;
    if (t_Stop < 1 || t_Stop > 20) t_Stop = 4;
    if (t_Net  < 1 || t_Net  > 20) t_Net  = 2;
    if (t_Twine< 1 || t_Twine > 20) t_Twine= 3;
  } else {
    // Первый запуск в жизни платы
    writeEEPROM_Long(ADDR_TOTAL_BALES, totalBales); writeEEPROM_Int(ADDR_SESSION_BALES, sessionBales);
    writeEEPROM_Byte(ADDR_T_DENS, t_Dens); writeEEPROM_Byte(ADDR_T_STOP, t_Stop);
    writeEEPROM_Byte(ADDR_T_NET, t_Net); writeEEPROM_Byte(ADDR_T_TWINE, t_Twine);
    writeEEPROM_Byte(ADDR_T_MOTOR, t_Motor); writeEEPROM_Byte(ADDR_SOUND_MODE, soundMode);
    writeEEPROM_Byte(ADDR_MAGIC, EEPROM_MAGIC_BYTE);
    saveRecoveryStatus(0); // Стартовый статус
  }

  // 2. Инициализация пинов
  pinMode(PIN_RS485_EN, OUTPUT); digitalWrite(PIN_RS485_EN, LOW); rs485.begin(9600); 
  densSensor.begin(); startSensor.begin(); endSensor.begin(); doorSensor.begin(); resetBtn.begin();
  pinMode(PIN_SWITCH_NET, INPUT_PULLUP); horn.begin(); beacon.begin();
  pinMode(PIN_RELAY_TWINE, OUTPUT); pinMode(PIN_RELAY_NET, OUTPUT);
  motorOff(); beacon.setMode(0); 
  
  // 3. УМНОЕ ВОССТАНОВЛЕНИЕ ПОСЛЕ СБОЯ ПИТАНИЯ (Smart Recovery)
  uint8_t recStatus = readEEPROM_Byte(ADDR_BALE_STATUS);
  uint8_t recChecksum = readEEPROM_Byte(ADDR_BALE_CHECKSUM);
  
  if ((uint8_t)(recStatus + recChecksum) == 255) {
    // Данные идеальны, помех не было. Восстанавливаем процесс!
    if (recStatus == 1) {
      // Сбой во время мотания шпагата (Самотек)
      // МОТОР НЕ ВКЛЮЧАЕМ! Просто ждем конца обвязки 90 сек.
      currentState = WAIT_END_SENSOR; 
      stateTimer = millis();
    } 
    else if (recStatus == 2) {
      // Сбой перед самой выгрузкой (Тюк уже замотан)
      currentState = WAIT_DOOR;
      doorWasOpened = false;
      horn.play(5, 200); // Громко просим выкинуть готовый тюк
    }
    else {
      // Статус 0 - всё спокойно, начинаем с чистого листа
      currentState = WAIT_DENSITY;
    }
  } else {
    // В памяти мусор (удар искры во время сохранения) - безопасно сбрасываем
    saveRecoveryStatus(0);
    currentState = WAIT_DENSITY;
  }

  // 4. ЗАПУСК АППАРАТНОГО СТОРОЖЕВОГО ПСА НА 2 СЕКУНДЫ
  // (Защита от любых радиопомех и зависаний шины I2C)
  wdt_enable(WDTO_2S); 
}


// ================= ОСНОВНОЙ ЦИКЛ (LOOP) =================
void loop() {
  wdt_reset(); // Гладим пса каждую итерацию (Я жив!)
  
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.clearWireTimeoutFlag(); // Сброс флага ошибки шины I2C
  #endif

  densSensor.update(); startSensor.update(); endSensor.update(); doorSensor.update(); resetBtn.update();
  horn.update(); beacon.update(); listenRS485();

  // Защита потери связи с пультом
  if (isRemoteConnected && (millis() - lastMasterPacketTime > 2000)) { isRemoteConnected = false; }
  
  // Аварийная кнопка на самом прессе (PIN_RESET). На этой сборке физически
  // не выведена (пин подтянут внутренней pullup, событие никогда не
  // сработает) - блок ниже безопасно неактивен, оставлен на случай, если
  // когда-нибудь добавишь кнопку.
  bool isResetJustPressed = resetBtn.justPressed();

  if (resetBtn.isHeldFor(10000) && currentState != TEST_MODE) {
    motorOff(); horn.play(1, 100); beacon.setMode(3); currentState = TEST_MODE; testModeStartTime = millis(); resetClicks = 0; 
  }
  if (isResetJustPressed && currentState != TEST_MODE) {
    if (millis() - lastResetClickTime <= 600) resetClicks++; else resetClicks = 1; 
    lastResetClickTime = millis();
    if (currentState != WAIT_DENSITY && currentState != RETURN_TO_HOME) { executeEmergencyReset(); } 
    else if (currentState == RETURN_TO_HOME) { motorOff(); horn.play(1, 600); beacon.setMode(0); currentState = WAIT_DENSITY; }
    if (resetClicks == 5 && currentState == WAIT_DENSITY) {
      resetClicks = 0; motorOn(); horn.play(1, 800); beacon.setMode(1); stateTimer = millis(); currentState = RETURN_TO_HOME;
    }
  }

  // === СБРОС ЦИКЛА ЧЕРЕЗ КОНЦЕВИК ДВЕРИ (по заводской схеме) ===
  // На заводе отдельной кнопки сброса не было - роль сброса выполняла кнопка,
  // подключённая параллельно концевику двери. Повторяем эту логику программно:
  // если дверь открылась, А ЦИКЛ ЕЩЁ НЕ ЗАВЕРШЁН (currentState != WAIT_DOOR),
  // это сигнал сброса - мотор глушим, статус восстановления обнуляем, тюк
  // НЕ засчитываем. Если же дверь открылась после полного завершения обвязки
  // (currentState == WAIT_DOOR) - работает обычная штатная выгрузка со счётом,
  // никакой сервисной логики (TEST_MODE, RETURN_TO_HOME) сюда не подключаем -
  // эти функции требуют отдельной физической кнопки, которой здесь нет.
  if (doorSensor.justPressed() && currentState != WAIT_DOOR && currentState != TEST_MODE) {
    executeEmergencyReset();
  }

  // === ЛОГИКА АВТОМАТА ===
  switch (currentState) {
    
    // ЭТАП 0: Ожидание нужной плотности тюка
    case WAIT_DENSITY: {
      uint32_t delayDens = (isRemoteConnected && rxData.isManualMode) ? 1000UL : (t_Dens * 1000UL);
      if (densSensor.isHeldFor(delayDens)) { 
        horn.play(3, 300); beacon.setMode(1); 
        stateTimer = millis(); 
        currentState = WAIT_TRACTOR; 
      } 
      break;
    }

    // ЭТАП 0.5: Ждем пока тракторист остановится (отработка t_Stop)
    case WAIT_TRACTOR:
      if (isRemoteConnected && rxData.isManualMode) {
        if (startSensor.isPressed()) { 
          horn.play(1, 200); stateTimer = millis(); 
          saveRecoveryStatus(1); // СОХРАНЕНИЕ 1: ПРОЦЕСС ПОШЕЛ
          currentState = MOTOR_RUNNING_TIMER; 
        }
      } else {
        if (millis() - stateTimer >= (t_Stop * 1000UL)) { 
          motorOn(); // Плюнули шпагат в камеру
          stateTimer = millis(); 
          saveRecoveryStatus(1); // СОХРАНЕНИЕ 1: ПРОЦЕСС ПОШЕЛ (Для автономного режима)
          currentState = WAIT_START_SENSOR; 
        } 
      }
      break;

    // ЭТАП 1: Мотор включен, ждем пока пойдет подача
    case WAIT_START_SENSOR:
      if (startSensor.isPressed()) { 
        horn.play(1, 200); stateTimer = millis(); 
        currentState = MOTOR_RUNNING_TIMER; 
      } 
      else if (t_Motor > 0 && millis() - stateTimer >= (t_Motor * 1000UL)) { 
        motorOff(); beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; 
      } 
      break;

    // ЭТАП 1.5: Мотор крутит заданное время t_Net или t_Twine
    case MOTOR_RUNNING_TIMER:
      if (millis() - stateTimer >= (getNetMode() ? (t_Net * 1000UL) : (t_Twine * 1000UL))) { 
        motorOff(); // Отключаем мотор, дальше механизм тянется "самотеком"
        stateTimer = millis(); 
        currentState = WAIT_END_SENSOR; 
      } 
      break;

    // ЭТАП 1.9: Ожидание пока нож отрубит шпагат (Самотек)
    case WAIT_END_SENSOR:
      if (endSensor.isPressed()) { 
        horn.play(2, 400); 
        doorWasOpened = false; 
        saveRecoveryStatus(2); // СОХРАНЕНИЕ 2: ТЮК ГОТОВ И ОТРЕЗАН
        currentState = WAIT_DOOR; 
      }
      // Защита от обрыва шпагата: если за 90 сек нож так и не упал - СБОЙ!
      else if (t_Motor > 0 && millis() - stateTimer >= (t_Motor * 1000UL)) { 
        beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; 
      } 
      break;

    // ЭТАП 2: Тюк замотан, ждем выгрузки (Открытия двери)
    case WAIT_DOOR:
      if (doorSensor.isPressed() && !doorWasOpened) doorWasOpened = true;
      else if (!doorSensor.isPressed() && doorWasOpened) { 
        // ВАЖНО: чекпоинт обнуляем ПЕРВЫМ, до записи счётчиков.
        // Если сброс платы (например, помеха от двигателя) случится
        // посреди этой последовательности - лучше потерять один тюк из
        // статистики (счётчик не успеет записаться), чем оставить систему
        // в "зависшем" WAIT_DOOR после перезапуска, где она будет
        // игнорировать плотность, ожидая выгрузки уже выброшенного тюка.
        saveRecoveryStatus(0); // СОХРАНЕНИЕ 0: КАМЕРА ПУСТА
        horn.play(2, 150); 
        totalBales++; sessionBales++; 
        writeEEPROM_Long(ADDR_TOTAL_BALES, totalBales); 
        writeEEPROM_Int(ADDR_SESSION_BALES, sessionBales);
        doorWasOpened = false; 
        beacon.setMode(0); 
        currentState = WAIT_DENSITY; 
      } 
      break;

    // СОСТОЯНИЯ ОШИБКИ И ТЕСТОВ
    case ERROR_STATE:
      if (millis() - stateTimer >= 4000) { horn.play(2, 200); stateTimer = millis(); } 
      break;
      
    case RETURN_TO_HOME:
      if (endSensor.isPressed()) { motorOff(); horn.play(2, 400); beacon.setMode(0); currentState = WAIT_DENSITY; }
      else if (t_Motor > 0 && millis() - stateTimer >= (t_Motor * 1000UL)) { motorOff(); beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; } 
      break;
      
    case TEST_MODE:
      if (millis() - testModeStartTime >= 60000) { horn.play(2, 400); beacon.setMode(0); doorWasOpened = doorSensor.isPressed(); currentState = WAIT_DENSITY; } 
      else { if (!horn.isBusy()) { digitalWrite(PIN_RELAY_SOUND, (densSensor.isPressed() || startSensor.isPressed() || endSensor.isPressed() || doorSensor.isPressed() || resetBtn.isPressed()) ? (cfg.soundActiveHigh ? HIGH : LOW) : (cfg.soundActiveHigh ? LOW : HIGH)); } } 
      break;
  }
}
