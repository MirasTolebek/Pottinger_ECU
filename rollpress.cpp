/*
 * ПРОЕКТ: Система управления пресс-подборщиком (Блок Пресса / Slave)
 * ВЕРСИЯ: 2.1 (Оптимизация EEPROM (update), полная валидация, фикс кликов)
 */

#include <SoftwareSerial.h>
#include <Wire.h>     

#define PIN_DENSITY        2    
#define PIN_S_START        3    
#define PIN_S_END          4    
#define PIN_DOOR           5    
#define PIN_RELAY_TWINE    6    
#define PIN_RELAY_SOUND    7    
#define PIN_RESET          8    
#define PIN_RELAY_LIGHT    9    
#define PIN_RELAY_NET      10   
#define PIN_SWITCH_NET     11   

#define PIN_RS485_RX       A0   
#define PIN_RS485_TX       A1   
#define PIN_RS485_EN       A2   

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// ВНЕШНИЙ EEPROM (I2C AT24Cxx)
#define EEPROM_ADDR 0x50       
#define EEPROM_MAGIC_BYTE 0x45 
#define ADDR_MAGIC        0    
#define ADDR_TOTAL_BALES  1    
#define ADDR_SESSION_BALES 5   
#define ADDR_T_DENS       7    
#define ADDR_T_STOP       8    
#define ADDR_T_NET        9    
#define ADDR_T_TWINE      10   
#define ADDR_SOUND_MODE   11   
#define ADDR_T_MOTOR      12   

uint32_t totalBales = 0;
uint16_t sessionBales = 0; 
uint8_t t_Dens = 1;   
uint8_t t_Stop = 4;
uint8_t t_Net = 2;
uint8_t t_Twine = 3;
uint8_t t_Motor = 10; 
uint8_t soundMode = 2; 

struct Config {
  const uint32_t debounce = 50;             
  const bool motorActiveHigh = true;  
  const bool soundActiveHigh = true;  
  const bool lightActiveHigh = true;  
};
Config cfg;

// СТРУКТУРЫ ДАННЫХ ДЛЯ ОБМЕНА
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

// =================================================================================
// ФУНКЦИИ РАБОТЫ С EEPROM (С ЗАЩИТОЙ ОТ ИЗНОСА)
// =================================================================================
void writeEEPROM_Byte(uint16_t mem_addr, uint8_t data) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(mem_addr >> 8));   
  Wire.write((int)(mem_addr & 0xFF)); 
  Wire.write(data);                   
  Wire.endTransmission(); delay(5); 
}

uint8_t readEEPROM_Byte(uint16_t mem_addr) {
  uint8_t data = 0xFF; Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(mem_addr >> 8)); Wire.write((int)(mem_addr & 0xFF)); Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 1); if (Wire.available()) data = Wire.read(); return data;
}

// НОВОЕ: Читаем перед записью. Экономит 5мс и бережет память!
void updateEEPROM_Byte(uint16_t mem_addr, uint8_t data) {
  if (readEEPROM_Byte(mem_addr) == data) return; 
  writeEEPROM_Byte(mem_addr, data);
}

void writeEEPROM_Int(uint16_t mem_addr, uint16_t data) {
  updateEEPROM_Byte(mem_addr, (data & 0xFF)); 
  updateEEPROM_Byte(mem_addr + 1, ((data >> 8) & 0xFF));
}

uint16_t readEEPROM_Int(uint16_t mem_addr) {
  uint16_t data = readEEPROM_Byte(mem_addr); data |= ((uint16_t)readEEPROM_Byte(mem_addr + 1) << 8); return data;
}

void writeEEPROM_Long(uint16_t mem_addr, uint32_t data) {
  updateEEPROM_Byte(mem_addr, (data & 0xFF)); 
  updateEEPROM_Byte(mem_addr + 1, ((data >> 8) & 0xFF));
  updateEEPROM_Byte(mem_addr + 2, ((data >> 16) & 0xFF)); 
  updateEEPROM_Byte(mem_addr + 3, ((data >> 24) & 0xFF));
}

uint32_t readEEPROM_Long(uint16_t mem_addr) {
  uint32_t data = readEEPROM_Byte(mem_addr); data |= ((uint32_t)readEEPROM_Byte(mem_addr + 1) << 8);
  data |= ((uint32_t)readEEPROM_Byte(mem_addr + 2) << 16); data |= ((uint32_t)readEEPROM_Byte(mem_addr + 3) << 24);
  return data;
}

// =================================================================================
// КЛАССЫ ПЕРИФЕРИИ
// =================================================================================
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
      if (isRemoteConnected && soundMode == 1) { shouldPlay = false; }
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
void motorOff() { 
  digitalWrite(PIN_RELAY_TWINE, cfg.motorActiveHigh ? LOW : HIGH);
  digitalWrite(PIN_RELAY_NET, cfg.motorActiveHigh ? LOW : HIGH);
}
void executeEmergencyReset() {
  if (currentState != WAIT_DENSITY && currentState != TEST_MODE) {  
    motorOff(); horn.play(1, 600); beacon.setMode(0); doorWasOpened = doorSensor.isPressed(); 
    resetClicks = 0; // ИСПРАВЛЕНИЕ: Сбрасываем счетчик кликов при аварии
    currentState = WAIT_DENSITY;           
  }
}

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

          if (rxData.resetSession) {
             sessionBales = 0; writeEEPROM_Int(ADDR_SESSION_BALES, sessionBales);
          }

          if (rxData.saveSettings) {
            t_Dens = rxData.timeoutDens; t_Stop = rxData.timeoutStop; t_Net = rxData.timeoutNet;
            t_Twine = rxData.timeoutTwine; t_Motor = rxData.timeoutMotor; soundMode = rxData.soundMode; 
            
            // Запись через умную функцию update (экономия памяти и времени)
            updateEEPROM_Byte(ADDR_T_DENS, t_Dens); updateEEPROM_Byte(ADDR_T_STOP, t_Stop);
            updateEEPROM_Byte(ADDR_T_NET, t_Net); updateEEPROM_Byte(ADDR_T_TWINE, t_Twine);
            updateEEPROM_Byte(ADDR_T_MOTOR, t_Motor); updateEEPROM_Byte(ADDR_SOUND_MODE, soundMode); 
          }
          if (rxData.doReset) executeEmergencyReset(); 
          sendRS485Reply(); 
        }
      }
    }
  }
}

void setup() {
  Wire.begin(); 
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true);
  #endif

  uint8_t magic = readEEPROM_Byte(ADDR_MAGIC);
  if (magic == EEPROM_MAGIC_BYTE) {
    totalBales = readEEPROM_Long(ADDR_TOTAL_BALES);
    sessionBales = readEEPROM_Int(ADDR_SESSION_BALES);
    t_Dens = readEEPROM_Byte(ADDR_T_DENS); t_Stop = readEEPROM_Byte(ADDR_T_STOP);
    t_Net = readEEPROM_Byte(ADDR_T_NET); t_Twine = readEEPROM_Byte(ADDR_T_TWINE);
    t_Motor = readEEPROM_Byte(ADDR_T_MOTOR); soundMode = readEEPROM_Byte(ADDR_SOUND_MODE);
    
    // ИСПРАВЛЕНИЕ: Тотальная валидация параметров после чтения из памяти
    if (soundMode > 2) soundMode = 2; 
    if (t_Motor < 5 || t_Motor > 30) t_Motor = 10;
    if (t_Dens < 1 || t_Dens > 20) t_Dens = 1;
    if (t_Stop < 1 || t_Stop > 20) t_Stop = 4;
    if (t_Net  < 1 || t_Net  > 20) t_Net  = 2;
    if (t_Twine< 1 || t_Twine > 20) t_Twine= 3;
    
  } else {
    writeEEPROM_Long(ADDR_TOTAL_BALES, totalBales); writeEEPROM_Int(ADDR_SESSION_BALES, sessionBales);
    writeEEPROM_Byte(ADDR_T_DENS, t_Dens); writeEEPROM_Byte(ADDR_T_STOP, t_Stop);
    writeEEPROM_Byte(ADDR_T_NET, t_Net); writeEEPROM_Byte(ADDR_T_TWINE, t_Twine);
    writeEEPROM_Byte(ADDR_T_MOTOR, t_Motor); writeEEPROM_Byte(ADDR_SOUND_MODE, soundMode);
    writeEEPROM_Byte(ADDR_MAGIC, EEPROM_MAGIC_BYTE);
  }

  pinMode(PIN_RS485_EN, OUTPUT); digitalWrite(PIN_RS485_EN, LOW); rs485.begin(9600); 
  densSensor.begin(); startSensor.begin(); endSensor.begin(); doorSensor.begin(); resetBtn.begin();
  pinMode(PIN_SWITCH_NET, INPUT_PULLUP); 
  horn.begin(); beacon.begin();
  pinMode(PIN_RELAY_TWINE, OUTPUT); pinMode(PIN_RELAY_NET, OUTPUT);
  motorOff(); beacon.setMode(0); 
}

void loop() {
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.clearWireTimeoutFlag();
  #endif

  densSensor.update(); startSensor.update(); endSensor.update(); doorSensor.update(); resetBtn.update();
  horn.update(); beacon.update(); 
  listenRS485();

  if (isRemoteConnected && (millis() - lastMasterPacketTime > 2000)) { isRemoteConnected = false; }

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
    case WAIT_DENSITY: {
      uint32_t delayDens = (isRemoteConnected && rxData.isManualMode) ? 1000UL : (t_Dens * 1000UL);
      if (densSensor.isHeldFor(delayDens)) { horn.play(3, 300); beacon.setMode(1); stateTimer = millis(); currentState = WAIT_TRACTOR; } 
      break;
    }
    case WAIT_TRACTOR:
      if (isRemoteConnected && rxData.isManualMode) {
        if (startSensor.isPressed()) { horn.play(1, 200); stateTimer = millis(); currentState = MOTOR_RUNNING_TIMER; }
      } else {
        if (millis() - stateTimer >= (t_Stop * 1000UL)) { motorOn(); stateTimer = millis(); currentState = WAIT_START_SENSOR; } 
      }
      break;
    case WAIT_START_SENSOR:
      if (startSensor.isPressed()) { horn.play(1, 200); stateTimer = millis(); currentState = MOTOR_RUNNING_TIMER; } 
      else if (millis() - stateTimer >= (t_Motor * 1000UL)) { motorOff(); beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; } 
      break;
    case MOTOR_RUNNING_TIMER:
      if (millis() - stateTimer >= (getNetMode() ? (t_Net * 1000UL) : (t_Twine * 1000UL))) { motorOff(); stateTimer = millis(); currentState = WAIT_END_SENSOR; } 
      break;
    case WAIT_END_SENSOR:
      if (endSensor.isPressed()) { horn.play(2, 400); doorWasOpened = false; currentState = WAIT_DOOR; }
      else if (millis() - stateTimer >= (t_Motor * 1000UL)) { beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; } 
      break;
    case WAIT_DOOR:
      if (doorSensor.isPressed() && !doorWasOpened) doorWasOpened = true;
      else if (!doorSensor.isPressed() && doorWasOpened) { 
        horn.play(2, 150); 
        totalBales++; sessionBales++; 
        
        // Используем writeEEPROM_Long/Int, которые внутри теперь работают через update!
        // Экономит ~25-30мс на каждый тюк
        writeEEPROM_Long(ADDR_TOTAL_BALES, totalBales); writeEEPROM_Int(ADDR_SESSION_BALES, sessionBales);
        
        doorWasOpened = false; beacon.setMode(0); currentState = WAIT_DENSITY; 
      } 
      break;
    case ERROR_STATE:
      if (millis() - stateTimer >= 4000) { horn.play(2, 200); stateTimer = millis(); } 
      break;
    case RETURN_TO_HOME:
      if (endSensor.isPressed()) { motorOff(); horn.play(2, 400); beacon.setMode(0); currentState = WAIT_DENSITY; }
      else if (millis() - stateTimer >= (t_Motor * 1000UL)) { motorOff(); beacon.setMode(2); stateTimer = millis(); currentState = ERROR_STATE; } break;
    case TEST_MODE:
      // Выход только по таймеру (60с) для надежности проверок концевиков
      if (millis() - testModeStartTime >= 60000) { horn.play(2, 400); beacon.setMode(0); doorWasOpened = doorSensor.isPressed(); currentState = WAIT_DENSITY; } 
      else { if (!horn.isBusy()) { digitalWrite(PIN_RELAY_SOUND, (densSensor.isPressed() || startSensor.isPressed() || endSensor.isPressed() || doorSensor.isPressed() || resetBtn.isPressed()) ? (cfg.soundActiveHigh ? HIGH : LOW) : (cfg.soundActiveHigh ? LOW : HIGH)); } } break;
  }
}
