#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// =================================================================================
// НОВАЯ РАСПИНОВКА ПУЛЬТА
// =================================================================================
#define PIN_SW_MODE      4    // Тумблер АВТО/РУЧНОЙ (Остался на D4)
#define PIN_SW_NET       2    // Тумблер СЕТКА/ШПАГАТ (Теперь D2)
#define PIN_BTN_SCREEN   3    // Кнопка ЭКРАН (Теперь D3)
#define PIN_BTN_ACTION   11   // Кнопка ДЕЙСТВИЕ (Теперь D11)

#define PIN_RS485_RX     8    
#define PIN_RS485_TX     9    
#define PIN_RS485_EN     10   

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// --- ОБНОВЛЕННАЯ СТРУКТУРА ДАННЫХ ДЛЯ ОТПРАВКИ ---
struct MasterData { 
  bool doReset; 
  bool isManualMode; 
  bool isNetMode; 
};
MasterData txData = {false, false, false}; // Начальные значения

struct SlaveData {
  uint8_t currentState;  
  uint16_t totalBales;   
  uint16_t sessionBales; 
};
SlaveData slaveData = {0, 0, 0}; 

SlaveData lastSlaveData = {255, 0, 0}; 
bool lastResetCmd = false;
bool lastLinkState = true;

bool isConnected = false;        
unsigned long lastPollTime = 0;  

byte noLinkChar[8] = { 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b00000, 0b11111, 0b00000 };

class Button {
  private:
    uint8_t pin; unsigned long lastChange; bool state; bool lastReading; bool stateChanged;
  public:
    Button(uint8_t p) : pin(p), lastChange(0), state(false), lastReading(false), stateChanged(false) {}
    
    void begin() { 
      pinMode(pin, INPUT_PULLUP); 
    }
    
    void update() {
      stateChanged = false;
      bool reading = !digitalRead(pin); 
      
      if (reading != lastReading) lastChange = millis();
      
      if ((millis() - lastChange) > 50) {
        if (state != reading) { 
          state = reading; 
          stateChanged = true; 
        }
      }
      lastReading = reading;
    }
    
    bool isPressed() { return state; } 
    bool justPressed() { return (state == true && stateChanged == true); } 
    bool isHeldFor(uint32_t time) { return state && ((millis() - lastChange) >= time); } 
    bool hasChanged() { return stateChanged; } 
};

Button swMode(PIN_SW_MODE);
Button swNet(PIN_SW_NET);
Button btnScreen(PIN_BTN_SCREEN);
Button btnAction(PIN_BTN_ACTION);

bool resetCommandSent = false;       
unsigned long lastDisplayUpdate = 0; 
uint8_t screenPage = 0;              
unsigned long screenTimer = 0;       

void setup() {
  Serial.begin(115200); 
  Serial.println(F("=== MASTER REMOTE START ==="));

  pinMode(PIN_RS485_EN, OUTPUT); digitalWrite(PIN_RS485_EN, LOW); rs485.begin(9600); 
  
  swMode.begin(); swNet.begin(); btnScreen.begin(); btnAction.begin();
  
  lcd.init(); lcd.backlight(); lcd.createChar(0, noLinkChar); 
  lcd.setCursor(0, 0); lcd.print(F("BALER CONTROL")); lcd.setCursor(0, 1); lcd.print(F("SYSTEM START..."));
  delay(1000); lcd.clear();
}

void pollSlave() {
  // --- ОБНОВЛЯЕМ ДАННЫЕ ПЕРЕД ОТПРАВКОЙ ---
  txData.isManualMode = swMode.isPressed();
  txData.isNetMode = swNet.isPressed();

  while (rs485.available()) rs485.read();
  digitalWrite(PIN_RS485_EN, HIGH); delay(2); 
  rs485.write(0xBB); 
  uint8_t crc = 0; uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(MasterData); i++) { rs485.write(ptr[i]); crc ^= ptr[i]; }
  rs485.write(crc); rs485.flush(); digitalWrite(PIN_RS485_EN, LOW); 

  unsigned long waitStart = millis(); bool replied = false;
  
  while (millis() - waitStart < 35) {
    if (rs485.available() > 0) {
      if (rs485.read() == 0xAA) { 
        unsigned long pStart = millis();
        while (rs485.available() < sizeof(SlaveData) + 1) {
          if (millis() - pStart > 20) break;
        }
        
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

  if (isConnected != lastLinkState) {
    if (isConnected) Serial.println(F("RS485 -> LINK RESTORED"));
    else Serial.println(F("RS485 -> ERROR: NO LINK"));
    lastLinkState = isConnected;
  }

  if (replied) {
    bool slaveChanged = (slaveData.currentState != lastSlaveData.currentState) ||
                        (slaveData.totalBales != lastSlaveData.totalBales) ||
                        (slaveData.sessionBales != lastSlaveData.sessionBales);
    bool txChanged = (txData.doReset != lastResetCmd);

    if (slaveChanged || txChanged) {
      Serial.print(F("RS485 UPDATE -> TX[Rst:")); Serial.print(txData.doReset);
      Serial.print(F("] | RX[St:")); Serial.print(slaveData.currentState);
      Serial.print(F(", Tot:")); Serial.print(slaveData.totalBales);
      Serial.print(F(", Ses:")); Serial.print(slaveData.sessionBales);
      Serial.println(F("]"));

      lastSlaveData = slaveData;
      lastResetCmd = txData.doReset;
    }
  }
}

void loop() {
  swMode.update(); swNet.update(); btnScreen.update(); btnAction.update();

  if (swMode.hasChanged()) {
    Serial.print(F("DEBUG: Switch MODE -> ")); Serial.println(swMode.isPressed() ? F("MANUAL [R]") : F("AUTO [A]"));
  }
  if (swNet.hasChanged()) {
    Serial.print(F("DEBUG: Switch NET -> ")); Serial.println(swNet.isPressed() ? F("NET [SET]") : F("TWINE [SHP]"));
  }
  if (btnScreen.hasChanged()) {
    Serial.print(F("DEBUG: Button SCREEN -> ")); Serial.println(btnScreen.isPressed() ? F("PRESSED") : F("RELEASED"));
  }
  if (btnAction.hasChanged()) {
    Serial.print(F("DEBUG: Button ACTION -> ")); Serial.println(btnAction.isPressed() ? F("PRESSED") : F("RELEASED"));
  }

  if (millis() - lastPollTime >= 300) { pollSlave(); lastPollTime = millis(); }

  if (btnScreen.justPressed()) {
    screenPage = (screenPage == 0) ? 1 : 0; 
    screenTimer = millis(); 
    lcd.clear();                            
    updateDisplay(); 
    lastDisplayUpdate = millis();
  }

  if (swMode.isPressed()) {
    // РУЧНОЙ РЕЖИМ
  } else {
    // АВТО РЕЖИМ
    if (btnAction.isHeldFor(1000) && !resetCommandSent) {
      Serial.println(F("DEBUG: HOLD 1 SEC! SENDING RESET COMMAND!"));
      txData.doReset = true; resetCommandSent = true; 
      screenPage = 0; lcd.clear(); lcd.setCursor(0, 0); lcd.print(F(">> SBROS OK <<  ")); delay(1000); lcd.clear();
    }
    if (!btnAction.isPressed()) resetCommandSent = false;
  }

  if (screenPage == 1 && (millis() - screenTimer >= 5000)) { screenPage = 0; lcd.clear(); }

  if (millis() - lastDisplayUpdate > 200) { updateDisplay(); lastDisplayUpdate = millis(); }
}

void updateDisplay() {
  if (screenPage == 0) {
    lcd.setCursor(0, 0);
    
    if (swMode.isPressed() && btnAction.isPressed()) { 
      lcd.print(F("PUSK MOTORA!   "));
    } else {
      switch (slaveData.currentState) {
        case 0: lcd.print(F("NABOR MASSY... ")); break; 
        case 1: lcd.print(F("STOP TRAKTOR!  ")); break; 
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

    lcd.setCursor(15, 0);
    if (!isConnected) lcd.write(0); else lcd.print(F(" ")); 

    lcd.setCursor(0, 1);
    char buffer[17]; char modesStr[9];
    
    strcpy(modesStr, swMode.isPressed() ? "[R]" : "[A]"); 
    strcat(modesStr, swNet.isPressed() ? "[SET]" : "[SHP]"); 
    
    sprintf(buffer, "%-8s%8u", modesStr, slaveData.sessionBales);
    lcd.print(buffer); 
  } 
  else if (screenPage == 1) {
    lcd.setCursor(0, 0); lcd.print(F("TOTAL RULONOV:  ")); 
    lcd.setCursor(15, 0); if (!isConnected) lcd.write(0);
    lcd.setCursor(0, 1);
    char buffer[17];
    sprintf(buffer, "      %-10u", slaveData.totalBales);
    lcd.print(buffer);
  }
}
