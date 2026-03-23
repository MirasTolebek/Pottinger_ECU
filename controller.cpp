#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h> // Защита от зависаний

LiquidCrystal_I2C lcd(0x27, 16, 2);

// =================================================================================
// РАСПИНОВКА ПУЛЬТА
// =================================================================================
#define PIN_SW_MODE      4    // Тумблер АВТО/РУЧНОЙ (D4)
#define PIN_SW_NET       2    // Тумблер СЕТКА/ШПАГАТ (D2)
#define PIN_BTN_SCREEN   3    // Кнопка ЭКРАН (D3)
#define PIN_BTN_ACTION   11   // Кнопка ДЕЙСТВИЕ (D11)

#define PIN_RS485_RX     8    
#define PIN_RS485_TX     9    
#define PIN_RS485_EN     10   

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// --- СТРУКТУРЫ ДАННЫХ ДЛЯ ОТПРАВКИ ---
struct MasterData { 
  bool doReset; 
  bool isManualMode; 
  bool isNetMode; 
  bool saveSettings; // Флаг команды "Сохрани настройки"
  uint8_t timeoutDens;
  uint8_t timeoutStop;
  uint8_t timeoutNet;
  uint8_t timeoutTwine;
};
MasterData txData = {false, false, false, false, 1, 4, 2, 3}; 

struct SlaveData {
  uint8_t currentState;  
  uint16_t sessionBales; 
  uint32_t totalBales;   
  uint8_t t_Dens;
  uint8_t t_Stop;
  uint8_t t_Net;
  uint8_t t_Twine;
};
SlaveData slaveData = {0, 0, 0, 1, 4, 2, 3}; 
SlaveData lastSlaveData = {255, 0, 0, 0, 0, 0, 0}; 

bool isConnected = false;        
unsigned long lastPollTime = 0;  

byte noLinkChar[8] = { 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b00000, 0b11111, 0b00000 };

class Button {
  private:
    uint8_t pin; unsigned long lastChange; bool state; bool lastReading; bool stateChanged;
  public:
    Button(uint8_t p) : pin(p), lastChange(0), state(false), lastReading(false), stateChanged(false) {}
    void begin() { pinMode(pin, INPUT_PULLUP); }
    void update() {
      stateChanged = false;
      bool reading = !digitalRead(pin); 
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

Button swMode(PIN_SW_MODE);
Button swNet(PIN_SW_NET);
Button btnScreen(PIN_BTN_SCREEN);
Button btnAction(PIN_BTN_ACTION);

bool resetCommandSent = false;       
unsigned long lastDisplayUpdate = 0; 

// Управление экранами: 0 - Главный, 1 - Тотал, 2 - Настройки
uint8_t screenPage = 0;              
unsigned long screenTimer = 0;       

// Переменные для секундомера (калибровка плотности)
unsigned long stopwatchStartTime = 0;
bool isStopwatchRunning = false;
uint16_t stoppedTimeSec = 0;
bool showStoppedTime = false;
uint8_t prevLoopState = 0;

// Переменные для Меню Настроек (Локальные, пока мы их редактируем)
unsigned long comboTimer = 0;
bool comboTriggered = false;
uint8_t settingIndex = 0; // 0=Плотность, 1=Трактор, 2=Сетка, 3=Шпагат
uint8_t edit_t_Dens, edit_t_Stop, edit_t_Net, edit_t_Twine;
bool pendingSave = false; // Флаг, что нужно отправить пакет сохранения

void setup() {
  Serial.begin(115200); 
  Serial.println(F("=== MASTER REMOTE START ==="));

  pinMode(PIN_RS485_EN, OUTPUT); digitalWrite(PIN_RS485_EN, LOW); rs485.begin(9600); 
  swMode.begin(); swNet.begin(); btnScreen.begin(); btnAction.begin();
  
  lcd.init(); lcd.backlight(); lcd.createChar(0, noLinkChar); 
  lcd.setCursor(0, 0); lcd.print(F("BALER CONTROL")); lcd.setCursor(0, 1); lcd.print(F("SYSTEM START..."));
  delay(1000); lcd.clear();

  wdt_enable(WDTO_2S); // Включаем защиту от зависаний (2 секунды)
}

void pollSlave() {
  txData.isManualMode = swMode.isPressed();
  txData.isNetMode = swNet.isPressed();

  if (pendingSave) {
    txData.saveSettings = true;
  }

  while (rs485.available()) rs485.read();
  digitalWrite(PIN_RS485_EN, HIGH); delay(2); 
  rs485.write(0xBB); 
  uint8_t crc = 0; uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(MasterData); i++) { rs485.write(ptr[i]); crc ^= ptr[i]; }
  rs485.write(crc); rs485.flush(); digitalWrite(PIN_RS485_EN, LOW); 

  if (pendingSave) {
    txData.saveSettings = false;
    pendingSave = false;
  }

  unsigned long waitStart = millis(); bool replied = false;
  
  // УВЕЛИЧЕН ТАЙМАУТ С 35 ДО 80 МС для передачи больших пакетов!
  while (millis() - waitStart < 80) {
    if (rs485.available() > 0) {
      if (rs485.read() == 0xAA) { 
        unsigned long pStart = millis();
        // УВЕЛИЧЕН ТАЙМАУТ ВНУТРЕННЕГО ЦИКЛА ДО 40 МС
        while (rs485.available() < sizeof(SlaveData) + 1) {
          if (millis() - pStart > 40) break; 
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

  if (replied) {
    if (slaveData.currentState != lastSlaveData.currentState || slaveData.sessionBales != lastSlaveData.sessionBales) {
      Serial.print(F("RS485 -> St:")); Serial.print(slaveData.currentState);
      Serial.print(F(" Ses:")); Serial.print(slaveData.sessionBales);
      Serial.print(F(" Tot:")); Serial.println(slaveData.totalBales);
      lastSlaveData = slaveData;
    }
  }
}

void loop() {
  wdt_reset(); // Гладим сторожевую собаку

  swMode.update(); swNet.update(); btnScreen.update(); btnAction.update();

  if (millis() - lastPollTime >= 300) { pollSlave(); lastPollTime = millis(); }

  // --- ЛОГИКА СЕКУНДОМЕРА ДЛЯ РУЧНОГО РЕЖИМА ---
  if (swMode.isPressed()) {
    if (slaveData.currentState == 1 && prevLoopState == 0) {
      // Только что сработал датчик - запускаем секундомер!
      stopwatchStartTime = millis();
      isStopwatchRunning = true;
      showStoppedTime = false;
    }
    if (isStopwatchRunning && btnAction.justPressed()) {
      // Тракторист нажал кнопку пуска - останавливаем секундомер!
      stoppedTimeSec = (millis() - stopwatchStartTime) / 1000;
      isStopwatchRunning = false;
      showStoppedTime = true;
    }
    if (slaveData.currentState == 0) {
      // Сброс при новом цикле
      isStopwatchRunning = false;
      showStoppedTime = false;
    }
  } else {
    isStopwatchRunning = false;
    showStoppedTime = false;
  }
  prevLoopState = slaveData.currentState;

  // =================================================================================
  // ЛОГИКА МЕНЮ НАСТРОЕК (ВХОД И ВЫХОД)
  // =================================================================================
  if (btnScreen.isPressed() && btnAction.isPressed() && isConnected) {
    
    // Аппаратная защита: блокируем вход в меню, если включен ручной режим!
    if (swMode.isPressed()) {
      if (millis() - comboTimer >= 500) { // Показываем ошибку быстро
        lcd.clear(); lcd.setCursor(0,0); lcd.print(F("PEREVEDI V AVTO!")); delay(1000); lcd.clear();
        comboTimer = millis();
      }
    } else {
      // Тумблер в АВТО - можно безопасно заходить
      if (millis() - comboTimer >= 2000 && !comboTriggered) {
        comboTriggered = true;
        if (screenPage != 2) {
          // ВХОД В НАСТРОЙКИ
          screenPage = 2; settingIndex = 0;
          edit_t_Dens = slaveData.t_Dens;
          edit_t_Stop = slaveData.t_Stop;
          edit_t_Net = slaveData.t_Net;
          edit_t_Twine = slaveData.t_Twine;
          lcd.clear(); lcd.setCursor(0,0); lcd.print(F("== NASTROYKI ==")); delay(1000); lcd.clear();
        } else {
          // ВЫХОД И СОХРАНЕНИЕ
          txData.timeoutDens = edit_t_Dens;
          txData.timeoutStop = edit_t_Stop;
          txData.timeoutNet = edit_t_Net;
          txData.timeoutTwine = edit_t_Twine;
          pendingSave = true; 
          
          screenPage = 0;
          lcd.clear(); lcd.setCursor(0,0); lcd.print(F("SOHRANENO V PRES")); delay(1000); lcd.clear();
        }
      }
    }
  } else {
    comboTimer = millis(); 
    comboTriggered = false;
  }

  // =================================================================================
  // ЛОГИКА НАВИГАЦИИ ПО ЭКРАНАМ
  // =================================================================================
  if (screenPage == 2) { 
    // --- ВНУТРИ МЕНЮ НАСТРОЕК ---
    if (btnScreen.justPressed() && !btnAction.isPressed()) {
      settingIndex++;
      if (settingIndex > 3) settingIndex = 0;
      lcd.clear();
    }

    if (btnAction.justPressed() && !btnScreen.isPressed()) {
      // Дополнительная защита внутри меню: если тракторист прямо в меню переключит на РУЧНОЙ
      if (swMode.isPressed()) {
        lcd.setCursor(0, 1); lcd.print(F("PEREVEDI V AVTO!")); delay(1000); lcd.clear();
      } else {
        uint8_t *valPtr;
        if (settingIndex == 0) valPtr = &edit_t_Dens;
        else if (settingIndex == 1) valPtr = &edit_t_Stop;
        else if (settingIndex == 2) valPtr = &edit_t_Net;
        else if (settingIndex == 3) valPtr = &edit_t_Twine;

        // --- ИСПОЛЬЗУЕМ ТУМБЛЕР СЕТКА/ШПАГАТ ДЛЯ +/- ---
        if (!swNet.isPressed()) { // ШПАГАТ (Разомкнут) -> Увеличить
          if (*valPtr < 20) (*valPtr)++;
        } else {                  // СЕТКА (Замкнут) -> Уменьшить
          if (*valPtr > 1) (*valPtr)--;
        }
      }
    }
  } else {
    // --- ОБЫЧНЫЙ РЕЖИМ ---
    if (btnScreen.justPressed() && !btnAction.isPressed()) {
      screenPage = (screenPage == 0) ? 1 : 0; 
      screenTimer = millis(); 
      lcd.clear();                            
      updateDisplay(); 
      lastDisplayUpdate = millis();
    }

    if (!swMode.isPressed()) {
      // АВТО РЕЖИМ (Кнопка работает как сброс)
      if (btnAction.isHeldFor(1000) && !resetCommandSent && !btnScreen.isPressed()) {
        txData.doReset = true; resetCommandSent = true; 
        screenPage = 0; lcd.clear(); lcd.setCursor(0, 0); lcd.print(F(">> SBROS OK <<  ")); delay(1000); lcd.clear();
      }
      if (!btnAction.isPressed()) resetCommandSent = false;
    }
    
    // Автовозврат с Тотала на Главный экран
    if (screenPage == 1 && (millis() - screenTimer >= 5000)) { screenPage = 0; lcd.clear(); }
  }

  // Отрисовка
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

    // Отрисовка секундомера в правом верхнем углу (Поверх текста)
    if (swMode.isPressed() && (isStopwatchRunning || showStoppedTime)) {
      lcd.setCursor(10, 0);
      char swBuf[6];
      sprintf(swBuf, "%2us  ", isStopwatchRunning ? (millis() - stopwatchStartTime) / 1000 : stoppedTimeSec);
      lcd.print(swBuf);
    }

    lcd.setCursor(15, 0);
    if (!isConnected) lcd.write(0); else lcd.print(F(" ")); 

    lcd.setCursor(0, 1);
    char buffer[17]; char modesStr[9];
    
    strcpy(modesStr, swMode.isPressed() ? "[R]" : "[A]"); 
    strcat(modesStr, swNet.isPressed() ? "[SET]" : "[SHP]"); 
    
    sprintf(buffer, "%-8s%8u", modesStr, slaveData.sessionBales);
    lcd.print(buffer); 

  } else if (screenPage == 1) {
    lcd.setCursor(0, 0); lcd.print(F("TOTAL RULONOV:  ")); 
    lcd.setCursor(15, 0); if (!isConnected) lcd.write(0);
    lcd.setCursor(0, 1);
    char buffer[17];
    sprintf(buffer, "      %-10lu", slaveData.totalBales); 
    lcd.print(buffer);

  } else if (screenPage == 2) {
    // Отрисовка Меню Настроек
    lcd.setCursor(0, 0);
    lcd.print(F("SET: "));
    switch (settingIndex) {
      case 0: lcd.print(F("PLOTNOST  ")); break;
      case 1: lcd.print(F("TRACTOR   ")); break;
      case 2: lcd.print(F("SETKA     ")); break;
      case 3: lcd.print(F("SHPAGAT   ")); break;
    }
    
    lcd.setCursor(0, 1);
    lcd.print(F("VREMYA: "));
    uint8_t val = 0;
    if (settingIndex == 0) val = edit_t_Dens;
    if (settingIndex == 1) val = edit_t_Stop;
    if (settingIndex == 2) val = edit_t_Net;
    if (settingIndex == 3) val = edit_t_Twine;
    
    char buffer[10];
    sprintf(buffer, "%2u sek  ", val);
    lcd.print(buffer);
  }
}
