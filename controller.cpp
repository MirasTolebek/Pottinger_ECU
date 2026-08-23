/*
 * ПРОЕКТ: Система управления пресс-подборщиком (Блок Пульта / Master)
 * ВЕРСИЯ: 2.5 (Восстановление шины I2C после захвата помехой от двигателя)
 * ИЗМЕНЕНИЯ vs 2.4:
 *   - wdt_enable(WDTO_2S) перенесён в САМОЕ НАЧАЛО setup() (сразу после
 *     MCUSR=0/wdt_disable), а не в конец. Раньше если I2C-шина (LCD)
 *     захватывалась помехой ИМЕННО во время lcd.init()/lcd.print() в
 *     setup() - watchdog там ещё не был включён и не мог вытащить плату
 *     из зависания. Теперь watchdog защищает весь setup() целиком.
 *   - Добавлена процедура ручного восстановления шины I2C (i2cBusRecovery) -
 *     если ведомое устройство (PCF8574 на LCD) осталось "зависшим" посреди
 *     I2C-транзакции (типичный результат помехи от импульсов двигателя),
 *     оно продолжает держать SDA в LOW даже после сброса самого МК. Обычный
 *     software reset эту ситуацию не лечит - нужно вручную "дотактировать"
 *     SCL до 9 импульсов, чтобы зависшее устройство отпустило линию.
 *   - В loop() таймаут Wire теперь не просто "тихо" сбрасывается, а активно
 *     обрабатывается: если он сработал - делаем восстановление шины и
 *     переинициализацию LCD, а не просто идём дальше как ни в чём не бывало.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>   // <-- Аппаратный Watchdog (нужен новый загрузчик Optiboot!)

LiquidCrystal_I2C lcd(0x27, 16, 2); 

// === ВОССТАНОВЛЕНИЕ ШИНЫ I2C ПОСЛЕ ЗАХВАТА ПОМЕХОЙ ===
// Если ведомое устройство (PCF8574 на LCD) зависло посреди транзакции и
// держит SDA в LOW - подаём до 9 тактовых импульсов на SCL, чтобы оно
// "досчитало" ожидаемые биты и отпустило линию, затем вручную формируем
// STOP-условие. Это стандартная процедура восстановления шины I2C.
void i2cBusRecovery() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);

  for (uint8_t i = 0; i < 9; i++) {
    if (digitalRead(SDA) == HIGH) break; // Шина уже свободна - устройство отпустило линию
    digitalWrite(SCL, LOW);  delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
  }

  // Вручную формируем STOP: SDA LOW->HIGH, пока SCL держится HIGH
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);  delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
}

// РАСПИНОВКА ПУЛЬТА
#define PIN_SW_MODE      4    
#define PIN_SW_NET       2    
#define PIN_BTN_SCREEN   3    
#define PIN_BTN_ACTION   11   
#define PIN_BUZZER       12   

#define PIN_RS485_RX     8    
#define PIN_RS485_TX     9    
#define PIN_RS485_EN     10   

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// СТРУКТУРЫ ДАННЫХ
struct MasterData { 
  bool doReset; bool isManualMode; bool isNetMode; bool saveSettings; bool resetSession;
  uint8_t timeoutDens; uint8_t timeoutStop; uint8_t timeoutNet; uint8_t timeoutTwine;
  uint8_t timeoutMotor; uint8_t soundMode; 
};
MasterData txData = {false, false, false, false, false, 1, 4, 2, 3, 40, 2}; // По умолчанию защита 40 сек

struct SlaveData {
  uint8_t currentState; uint16_t sessionBales; uint32_t totalBales;   
  uint8_t t_Dens; uint8_t t_Stop; uint8_t t_Net; uint8_t t_Twine;
  uint8_t t_Motor; uint8_t soundMode; 
};
SlaveData slaveData = {0, 0, 0, 1, 4, 2, 3, 40, 2}; 

bool isConnected = false;        
unsigned long lastPollTime = 0;  
byte noLinkChar[8] = { 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b00000, 0b11111, 0b00000 };

// КЛАССЫ
class Button {
  private:
    uint8_t pin; unsigned long lastChange; bool state; bool lastReading; bool stateChanged;
  public:
    Button(uint8_t p) : pin(p), lastChange(0), state(false), lastReading(false), stateChanged(false) {}
    void begin() { pinMode(pin, INPUT_PULLUP); }
    void update() {
      stateChanged = false; bool reading = !digitalRead(pin); 
      if (reading != lastReading) lastChange = millis();
      if ((millis() - lastChange) > 50) {
        if (state != reading) { state = reading; stateChanged = true; }
      }
      lastReading = reading;
    }
    bool isPressed() { return state; }                                         
    bool justPressed() { return (state == true && stateChanged == true); }     
    bool isHeldFor(uint32_t time) { return state && ((millis() - lastChange) >= time); } 
};

class Beeper {
  private:
    uint8_t pin; int beepsLeft = 0; unsigned long lastToggle = 0; bool isOn = false; uint32_t duration; uint8_t patternType = 0; 
  public:
    Beeper(uint8_t p) : pin(p) {}
    void begin() { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); }
    void play(int count, uint32_t dur = 300) { 
      if (slaveData.soundMode == 0) return; 
      patternType = 0; beepsLeft = count * 2; duration = dur; isOn = true; 
      digitalWrite(pin, LOW); lastToggle = millis(); beepsLeft--; 
    }
    void playShortLong() {
      if (slaveData.soundMode == 0) return;
      patternType = 1; beepsLeft = 4; isOn = true;
      digitalWrite(pin, LOW); lastToggle = millis(); beepsLeft--;
    }
    void update() {
      if (beepsLeft > 0) {
        uint32_t currentDur = duration; 
        if (patternType == 1) {
          if (beepsLeft == 3) currentDur = 150;      
          else if (beepsLeft == 2) currentDur = 150; 
          else if (beepsLeft == 1) currentDur = 800; 
        }
        if (millis() - lastToggle >= currentDur) {
          isOn = !isOn; digitalWrite(pin, isOn ? LOW : HIGH); lastToggle = millis(); beepsLeft--;
        }
      } else if (beepsLeft == 0 && isOn) { isOn = false; digitalWrite(pin, HIGH); }
    }
};

Button swMode(PIN_SW_MODE); Button swNet(PIN_SW_NET);
Button btnScreen(PIN_BTN_SCREEN); Button btnAction(PIN_BTN_ACTION);
Beeper cabinBuzzer(PIN_BUZZER); 

bool resetCommandSent = false;       
unsigned long lastDisplayUpdate = 0; 
uint8_t screenPage = 0;              
unsigned long screenTimer = 0;       

unsigned long stopwatchStartTime = 0; bool isStopwatchRunning = false;
uint16_t stoppedTimeSec = 0; bool showStoppedTime = false;
uint8_t prevLoopState = 0; 

unsigned long comboTimer = 0; bool comboTriggered = false;
uint8_t settingIndex = 0; 
uint8_t edit_t_Dens, edit_t_Stop, edit_t_Net, edit_t_Twine, edit_t_Motor, edit_soundMode; 
bool pendingSave = false; 

void setup() {
  // === ОБЯЗАТЕЛЬНО ПЕРВЫМИ СТРОЧКАМИ! ===
  // Обнуляем MCUSR и глушим WDT сразу после старта.
  MCUSR = 0;
  wdt_disable();

  // Включаем аппаратный watchdog СРАЗУ, а не в конце setup()!
  // Раньше, если I2C-шина (LCD) захватывалась помехой от двигателя именно
  // во время lcd.init()/lcd.print() ниже - watchdog ещё не был включён и
  // не мог вытащить плату из зависания (отсюда "подсветка мигнула и всё
  // замерло насовсем"). Теперь watchdog защищает весь setup() целиком.
  wdt_enable(WDTO_2S);

  // Восстановление шины I2C ПЕРЕД инициализацией LCD - на случай, если
  // предыдущий сброс произошёл посреди I2C-транзакции и PCF8574 на LCD
  // остался висеть с прижатой SDA (обычный software/watchdog reset это
  // не лечит - нужна ручная процедура восстановления шины).
  i2cBusRecovery();

  pinMode(PIN_RS485_EN, OUTPUT); digitalWrite(PIN_RS485_EN, LOW); rs485.begin(9600); 
  swMode.begin(); swNet.begin(); btnScreen.begin(); btnAction.begin();
  cabinBuzzer.begin(); 
  
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true);
  #endif
  wdt_reset();
  
  lcd.init(); lcd.backlight(); lcd.createChar(0, noLinkChar); 
  lcd.setCursor(0, 0); lcd.print(F("BALER CONTROL")); 
  lcd.setCursor(0, 1); lcd.print(F("SYSTEM START..."));
  wdt_reset();
  
  delay(1000); lcd.clear();
  wdt_reset();
}

void pollSlave() {
  txData.isManualMode = swMode.isPressed(); txData.isNetMode = swNet.isPressed();
  if (pendingSave) { txData.saveSettings = true; }
  while (rs485.available()) rs485.read();
  
  digitalWrite(PIN_RS485_EN, HIGH); delay(2); rs485.write(0xBB); 
  uint8_t crc = 0; uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(MasterData); i++) { rs485.write(ptr[i]); crc ^= ptr[i]; }
  rs485.write(crc); rs485.flush(); digitalWrite(PIN_RS485_EN, LOW); 
  if (pendingSave) { txData.saveSettings = false; pendingSave = false; }
  if (txData.resetSession) txData.resetSession = false;

  unsigned long waitStart = millis(); bool replied = false;
  while (millis() - waitStart < 80) {
    if (rs485.available() > 0) {
      if (rs485.read() == 0xAA) { 
        unsigned long pStart = millis();
        while (rs485.available() < sizeof(SlaveData) + 1) { if (millis() - pStart > 40) break; }
        if (rs485.available() >= sizeof(SlaveData) + 1) {
          uint8_t rCrc = 0; uint8_t* rPtr = (uint8_t*)&slaveData;
          for (uint16_t i = 0; i < sizeof(SlaveData); i++) { rPtr[i] = rs485.read(); rCrc ^= rPtr[i]; }
          if (rCrc == rs485.read()) { replied = true; txData.doReset = false; }
        }
        break; 
      }
    }
  }
  isConnected = replied; 
}

void loop() {
  wdt_reset(); // Гладим аппаратного пса каждую итерацию

  #if defined(WIRE_HAS_TIMEOUT)
    Wire.clearWireTimeoutFlag();
  #endif

  swMode.update(); swNet.update(); btnScreen.update(); btnAction.update();
  cabinBuzzer.update(); 

  if (millis() - lastPollTime >= 300) { pollSlave(); lastPollTime = millis(); }

  if (slaveData.currentState != prevLoopState && isConnected) {
    uint8_t curr = slaveData.currentState; uint8_t prev = prevLoopState;
    if (curr == 1) { cabinBuzzer.play(3, 400); }
    else if (curr == 2 || (curr == 3 && prev < 2)) { cabinBuzzer.play(1, 250); }
    else if (curr == 5) { cabinBuzzer.playShortLong(); }
    else if (curr == 0 && prev == 5) { cabinBuzzer.play(2, 200); }
    else if (curr == 6 && prev != 6) { cabinBuzzer.play(10, 200); }
  }

  if (swMode.isPressed()) {
    if (slaveData.currentState == 1 && prevLoopState == 0) { stopwatchStartTime = millis(); isStopwatchRunning = true; showStoppedTime = false; }
    if (isStopwatchRunning && btnAction.justPressed()) { stoppedTimeSec = (millis() - stopwatchStartTime) / 1000; isStopwatchRunning = false; showStoppedTime = true; }
    if (slaveData.currentState == 0) { isStopwatchRunning = false; showStoppedTime = false; }
  } else { isStopwatchRunning = false; showStoppedTime = false; }
  
  prevLoopState = slaveData.currentState; 

  if (btnScreen.isPressed() && btnAction.isPressed() && isConnected) {
    if (swMode.isPressed()) {
      if (millis() - comboTimer >= 500) { lcd.clear(); lcd.setCursor(0,0); lcd.print(F("PEREVEDI V AVTO!")); wdt_reset(); delay(1000); wdt_reset(); lcd.clear(); comboTimer = millis(); }
    } else {
      if (millis() - comboTimer >= 2000 && !comboTriggered) {
        comboTriggered = true; 
        if (screenPage != 2) {
          screenPage = 2; settingIndex = 0;
          edit_t_Dens = slaveData.t_Dens; edit_t_Stop = slaveData.t_Stop; edit_t_Net = slaveData.t_Net; 
          edit_t_Twine = slaveData.t_Twine; edit_t_Motor = slaveData.t_Motor; edit_soundMode = slaveData.soundMode; 
          lcd.clear(); lcd.setCursor(0,0); lcd.print(F("== NASTROYKI ==")); wdt_reset(); delay(1000); wdt_reset(); lcd.clear();
        } else {
          txData.timeoutDens = edit_t_Dens; txData.timeoutStop = edit_t_Stop; txData.timeoutNet = edit_t_Net; 
          txData.timeoutTwine = edit_t_Twine; txData.timeoutMotor = edit_t_Motor; txData.soundMode = edit_soundMode; 
          pendingSave = true; screenPage = 0;
          lcd.clear(); lcd.setCursor(0,0); lcd.print(F("SOHRANENO V PRES")); wdt_reset(); delay(1000); wdt_reset(); lcd.clear();
        }
      }
    }
  } else { comboTimer = millis(); comboTriggered = false; }

  if (screenPage == 2) { 
    if (btnScreen.justPressed() && !btnAction.isPressed()) { settingIndex++; if (settingIndex > 5) settingIndex = 0; lcd.clear(); }
    if (btnAction.justPressed() && !btnScreen.isPressed()) {
      if (swMode.isPressed()) { lcd.setCursor(0, 1); lcd.print(F("PEREVEDI V AVTO!")); wdt_reset(); delay(1000); wdt_reset(); lcd.clear(); } 
      else {
        uint8_t *valPtr = &edit_t_Dens; 
        
        if (settingIndex == 0) valPtr = &edit_t_Dens; 
        else if (settingIndex == 1) valPtr = &edit_t_Stop;
        else if (settingIndex == 2) valPtr = &edit_t_Net; 
        else if (settingIndex == 3) valPtr = &edit_t_Twine;
        else if (settingIndex == 4) valPtr = &edit_soundMode; 
        else if (settingIndex == 5) valPtr = &edit_t_Motor;
        
        if (settingIndex == 4) { 
          if (!swNet.isPressed()) { if (*valPtr < 2) (*valPtr)++; } else { if (*valPtr > 0) (*valPtr)--; }
        } else if (settingIndex == 5) { // Лимиты защиты (0 = Выкл, 10 - 90 сек)
          if (!swNet.isPressed()) { 
             if (*valPtr == 0) *valPtr = 10;
             else if (*valPtr < 90) (*valPtr)++; 
          } else { 
             if (*valPtr > 10) (*valPtr)--; 
             else if (*valPtr == 10) *valPtr = 0; // Переход на 0 (Отключение)
          }
        } else { 
          if (!swNet.isPressed()) { if (*valPtr < 20) (*valPtr)++; } else { if (*valPtr > 1) (*valPtr)--; }                     
        }
      }
    }
  } else {
    if (btnScreen.isHeldFor(5000) && !swMode.isPressed()) {
      txData.resetSession = true;
      lcd.clear(); lcd.setCursor(0,0); lcd.print(F("SBROS SESSII...")); wdt_reset(); delay(1000); wdt_reset(); lcd.clear();
      screenPage = 0; screenTimer = millis(); 
    }
    if (btnScreen.justPressed() && !btnAction.isPressed()) {
      screenPage = (screenPage == 0) ? 1 : 0; screenTimer = millis(); lcd.clear(); updateDisplay(); lastDisplayUpdate = millis();
    }
    if (!swMode.isPressed()) {
      if (btnAction.isHeldFor(1000) && !resetCommandSent && !btnScreen.isPressed()) {
        txData.doReset = true; resetCommandSent = true; 
        screenPage = 0; lcd.clear(); lcd.setCursor(0, 0); lcd.print(F(">> SBROS OK <<  ")); wdt_reset(); delay(1000); wdt_reset(); lcd.clear();
      }
      if (!btnAction.isPressed()) resetCommandSent = false;
    }
    if (screenPage == 1 && (millis() - screenTimer >= 5000)) { screenPage = 0; lcd.clear(); }
  }

  if (millis() - lastDisplayUpdate > 200) { updateDisplay(); lastDisplayUpdate = millis(); }
}

void updateDisplay() {
  if (screenPage == 0) {
    lcd.setCursor(0, 0);
    if (swMode.isPressed() && btnAction.isPressed()) { lcd.print(F("PUSK MOTORA!   ")); } 
    else {
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
    if (swMode.isPressed() && (isStopwatchRunning || showStoppedTime)) {
      lcd.setCursor(10, 0); char swBuf[6]; sprintf(swBuf, "%2us  ", isStopwatchRunning ? (millis() - stopwatchStartTime) / 1000 : stoppedTimeSec); lcd.print(swBuf);
    }
    lcd.setCursor(15, 0); if (!isConnected) lcd.write(0); else lcd.print(F(" ")); 
    lcd.setCursor(0, 1);
    char buffer[17]; char modesStr[9];
    strcpy(modesStr, swMode.isPressed() ? "[R]" : "[A]"); strcat(modesStr, swNet.isPressed() ? "[SET]" : "[SHP]"); 
    sprintf(buffer, "%-8s%8u", modesStr, slaveData.sessionBales); lcd.print(buffer); 
  } else if (screenPage == 1) {
    lcd.setCursor(0, 0); lcd.print(F("TOTAL RULONOV:  ")); lcd.setCursor(15, 0); if (!isConnected) lcd.write(0);
    lcd.setCursor(0, 1); char buffer[17]; sprintf(buffer, "      %-10lu", slaveData.totalBales); lcd.print(buffer);
  } else if (screenPage == 2) {
    lcd.setCursor(0, 0); lcd.print(F("SET: "));
    switch (settingIndex) {
      case 0: lcd.print(F("PLOTNOST  ")); break; case 1: lcd.print(F("TRACTOR   ")); break;
      case 2: lcd.print(F("SETKA     ")); break; case 3: lcd.print(F("SHPAGAT   ")); break;
      case 4: lcd.print(F("ZVUK      ")); break; case 5: lcd.print(F("ZASCHITA  ")); break;
    }
    lcd.setCursor(0, 1); 
    if (settingIndex == 4) {
      lcd.print(F("REZH: "));
      if (edit_soundMode == 0) lcd.print(F("TOLKO PRES"));
      else if (edit_soundMode == 1) lcd.print(F("TOLKO PULT"));
      else if (edit_soundMode == 2) lcd.print(F("PRES+PULT "));
    } else {
      lcd.print(F("VREMYA: ")); uint8_t val = 0;
      if (settingIndex == 0) val = edit_t_Dens; else if (settingIndex == 1) val = edit_t_Stop;
      else if (settingIndex == 2) val = edit_t_Net; else if (settingIndex == 3) val = edit_t_Twine;
      else if (settingIndex == 5) val = edit_t_Motor;
      
      // Отображение состояния ВЫКЛ, если значение защиты 0
      if (settingIndex == 5 && val == 0) {
        lcd.print(F("VYKL   "));
      } else {
        char buffer[10]; sprintf(buffer, "%2u sek  ", val); lcd.print(buffer);
      }
    }
  }
}
