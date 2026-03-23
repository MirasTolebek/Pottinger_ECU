#include <SoftwareSerial.h>

// =================================================================================
// РАСПИНОВКА (PINS) - БЛОК ПРЕССА
// =================================================================================
#define PIN_DENSITY        2    // Вход: Плотность (через оптрон)
#define PIN_S_START        3    // Вход: Начало обвязки (через оптрон)
#define PIN_S_END          4    // Вход: Конец обвязки (через оптрон)
#define PIN_DOOR           5    // Вход: Дверь открыта (через оптрон)
#define PIN_RELAY_TWINE    6    // Выход: Реле ШПАГАТА
#define PIN_RELAY_SOUND    7    // Выход: Реле гудка 
#define PIN_RESET          8    // Вход: Локальная кнопка Сброс
#define PIN_RELAY_LIGHT    9    // Выход: Реле маячка 
#define PIN_RELAY_NET      10   // Выход: Реле СЕТКИ
#define PIN_SWITCH_NET     11   // Вход: Тумблер Сетка/Шпагат на блоке

#define PIN_RS485_RX       A0   
#define PIN_RS485_TX       A1   
#define PIN_RS485_EN       A2   

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

struct Config {
  const uint32_t plotnostLozh = 1000;       
  const uint32_t timeForStopTraktor = 4000; 
  const uint32_t timeoutMotorMax = 10000;   
  const uint32_t timerTwineRun = 3000;      
  const uint32_t timerNetRun = 2000;        
  const uint32_t timeoutEndSensor = 15000;  
  const uint32_t timeoutReturnHome = 10000; 
  const uint32_t debounce = 50;             

  const bool motorActiveHigh = true;  
  const bool soundActiveHigh = false; 
  const bool lightActiveHigh = true;  
};
Config cfg;

// --- ОБНОВЛЕННАЯ СТРУКТУРА ДАННЫХ ОТ ПУЛЬТА ---
struct MasterData { 
  bool doReset; 
  bool isManualMode; // true = Ручной, false = Авто
  bool isNetMode;    // true = Сетка, false = Шпагат
};
MasterData rxData;

struct SlaveData {
  uint8_t currentState;  
  uint16_t totalBales;   
  uint16_t sessionBales; 
};
SlaveData txData = {0, 0, 0}; 

// Глобальные переменные для сторожевого таймера (Watchdog)
unsigned long lastMasterPacketTime = 0;
bool isRemoteConnected = false;

// --- КЛАССЫ ---
class Sensor {
  private:
    uint8_t pin; bool invertLogic; unsigned long lastChange; unsigned long stateChangeTime;
    bool state; bool lastReading; bool stateChanged;
  public:
    Sensor(uint8_t p, bool inv = true) : pin(p), invertLogic(inv), lastChange(0), stateChangeTime(0), state(false), lastReading(false), stateChanged(false) {}
    void begin() { pinMode(pin, invertLogic ? INPUT_PULLUP : INPUT); }
    void update() {
      stateChanged = false; 
      bool reading = digitalRead(pin);
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
    void play(int count, uint32_t dur = 300) { beepsLeft = count * 2; duration = dur; isRelayOn = true; turnOn(); lastToggle = millis(); beepsLeft--; }
    void update() {
      if (beepsLeft > 0 && (millis() - lastToggle >= duration)) {
        isRelayOn = !isRelayOn; if (isRelayOn) turnOn(); else turnOff();
        lastToggle = millis(); beepsLeft--;
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

enum BalerState { WAIT_DENSITY, WAIT_TRACTOR, WAIT_START_SENSOR, MOTOR_RUNNING_TIMER, WAIT_END_SENSOR, WAIT_DOOR, ERROR_STATE, TEST_MODE, RETURN_TO_HOME };
BalerState currentState = WAIT_DENSITY; 

unsigned long stateTimer = 0;        
unsigned long testModeStartTime = 0; 
bool doorWasOpened = false;          
uint8_t resetClicks = 0;             
unsigned long lastResetClickTime = 0;

// --- УМНЫЙ ВЫБОР МАТЕРИАЛА ---
bool getNetMode() { 
  if (isRemoteConnected) {
    return rxData.isNetMode; // Приоритет пульта
  } else {
    return !digitalRead(PIN_SWITCH_NET); // Локальный тумблер
  }
}

void motorOn() { 
  if (getNetMode()) digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? HIGH : LOW);
  else digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? HIGH : LOW);
}

void motorOff() { 
  digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? LOW : HIGH);
  digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? LOW : HIGH);
}

void executeEmergencyReset() {
  if (currentState != WAIT_DENSITY && currentState != TEST_MODE) {  
    motorOff(); horn.play(1, 600); beacon.setMode(0); doorWasOpened = doorSensor.isPressed(); currentState = WAIT_DENSITY;           
  }
}

void sendRS485Reply() {
  digitalWrite(PIN_RS485_EN, HIGH); delay(2);
  txData.currentState = currentState; 
  rs485.write(0xAA); 
  uint8_t crc = 0; uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(SlaveData); i++) { rs485.write(ptr[i]); crc ^= ptr[i]; }
  rs485.write(crc); 
  rs485.flush(); digitalWrite(PIN_RS485_EN, LOW); 
}

void listenRS485() {
  if (rs485.available() > 0) { 
    if (rs485.read() == 0xBB) { 
      delay(5); 
      if (rs485.available() >= sizeof(MasterData) + 1) { 
        uint8_t crc = 0; uint8_t* ptr = (uint8_t*)&rxData;
        for (uint16_t i = 0; i < sizeof(MasterData); i++) { ptr[i] = rs485.read(); crc ^= ptr[i]; }
        if (crc == rs485.read()) {
          // Успешный прием! Обновляем таймер сторожа.
          lastMasterPacketTime = millis();
          isRemoteConnected = true;

          if (rxData.doReset) executeEmergencyReset(); 
          sendRS485Reply();
        }
      }
    }
  }
}

void setup() {
  pinMode(PIN_RS485_EN, OUTPUT); digitalWrite(PIN_RS485_EN, LOW); rs485.begin(9600); 
  densSensor.begin(); startSensor.begin(); endSensor.begin(); doorSensor.begin(); resetBtn.begin();
  pinMode(PIN_SWITCH_NET, INPUT_PULLUP); 
  horn.begin(); beacon.begin();
  pinMode(PIN_RELAY_TWINE, OUTPUT); pinMode(PIN_RELAY_NET, OUTPUT);
  motorOff(); beacon.setMode(0); 
}

void loop() {
  densSensor.update(); startSensor.update(); endSensor.update(); doorSensor.update(); resetBtn.update();
  horn.update(); beacon.update(); 
  listenRS485();

  // --- ОБРАБОТКА СТОРОЖЕВОГО ТАЙМЕРА (Watchdog) ---
  // Если от пульта не было вестей 2 секунды - отключаем режим "Remote"
  if (isRemoteConnected && (millis() - lastMasterPacketTime > 2000)) {
    isRemoteConnected = false;
  }

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

  switch (currentState) {
    case WAIT_DENSITY:
      if (densSensor.isHeldFor(cfg.plotnostLozh)) { horn.play(3, 300); beacon.setMode(1); stateTimer = millis(); currentState = WAIT_TRACTOR; } break;
    
    case WAIT_TRACTOR:
      // ВЕТВЛЕНИЕ АВТО/РУЧНОЙ РЕЖИМ
      if (isRemoteConnected && rxData.isManualMode) {
        // РУЧНОЙ РЕЖИМ: Мы не запускаем мотор. Ждем, пока тракторист аппаратно подаст 12В на мотор.
        // Как только мотор дернет механизм, сработает датчик "Старт", и мы продолжим цикл.
        if (startSensor.isPressed()) {
          horn.play(1, 800); 
          stateTimer = millis(); 
          currentState = MOTOR_RUNNING_TIMER;
        }
      } else {
        // АВТО РЕЖИМ (Или пульт отключен): Пресс работает сам
        if (millis() - stateTimer >= cfg.timeForStopTraktor) { motorOn(); stateTimer = millis(); currentState = WAIT_START_SENSOR; } 
      }
      break;

    case WAIT_START_SENSOR:
      if (startSensor.isPressed()) { horn.play(1, 800); stateTimer = millis(); currentState = MOTOR_RUNNING_TIMER; } 
      else if (millis() - stateTimer >= cfg.timeoutMotorMax) { motorOff(); beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; } break;
    
    case MOTOR_RUNNING_TIMER:
      if (millis() - stateTimer >= (getNetMode() ? cfg.timerNetRun : cfg.timerTwineRun)) { motorOff(); stateTimer = millis(); currentState = WAIT_END_SENSOR; } break;
    
    case WAIT_END_SENSOR:
      if (endSensor.isPressed()) { horn.play(2, 400); doorWasOpened = false; currentState = WAIT_DOOR; }
      else if (millis() - stateTimer >= cfg.timeoutEndSensor) { beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; } break;
    
    case WAIT_DOOR:
      if (doorSensor.isPressed() && !doorWasOpened) doorWasOpened = true;
      else if (!doorSensor.isPressed() && doorWasOpened) { txData.totalBales++; txData.sessionBales++; doorWasOpened = false; beacon.setMode(0); currentState = WAIT_DENSITY; } break;
    
    case ERROR_STATE:
      if (millis() - stateTimer >= 4000) { horn.play(2, 200); stateTimer = millis(); } break;
    
    case RETURN_TO_HOME:
      if (endSensor.isPressed()) { motorOff(); horn.play(2, 400); beacon.setMode(0); currentState = WAIT_DENSITY; }
      else if (millis() - stateTimer >= cfg.timeoutReturnHome) { motorOff(); beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; } break;
    
    case TEST_MODE:
      if (millis() - testModeStartTime >= 60000) { horn.play(2, 400); beacon.setMode(0); doorWasOpened = doorSensor.isPressed(); currentState = WAIT_DENSITY; } 
      else { if (!horn.isBusy()) { digitalWrite(PIN_RELAY_SOUND, (densSensor.isPressed() || startSensor.isPressed() || endSensor.isPressed() || doorSensor.isPressed()) ? (cfg.soundActiveHigh ? HIGH : LOW) : (cfg.soundActiveHigh ? LOW : HIGH)); } } break;
  }
}
