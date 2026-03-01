#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// |||----------------------------|||-ПИНЫ ПУЛЬТА-|||----------------------------|||
#define PIN_SW_MODE      4    // Тумблер: LOW (GND) = АВТО, HIGH = РУЧНОЙ
#define PIN_SW_NET       5    // Тумблер: LOW (GND) = СЕТКА, HIGH = ШПАГАТ
#define PIN_BTN_SCREEN   6    // Кнопка 1: Переключение экранов (на GND)
#define PIN_BTN_ACTION   7    // Кнопка 2: Сброс / Ручной пуск (на GND)

// ПИНЫ ДЛЯ МОДУЛЯ MAX485
#define PIN_RS485_RX     8    // RO
#define PIN_RS485_TX     9    // DI
#define PIN_RS485_EN     10   // DE и RE (соединены вместе)

SoftwareSerial rs485(PIN_RS485_RX, PIN_RS485_TX);

// |||----------------------------|||-СТРУКТУРЫ ДАННЫХ RS485-|||----------------------------|||

// Пакет ДЛЯ отправки на Пресс (Slave)
struct MasterData {
  bool doReset; // Команда на сброс ошибки
};
MasterData txData = {false};

// Пакет ОТ Пресса (Slave)
struct SlaveData {
  uint8_t currentState;  
  uint16_t totalBales;   
  uint16_t sessionBales; 
};
SlaveData slaveData = {0, 0, 0}; // Стартуем с нулей

bool isConnected = false; 
unsigned long lastPollTime = 0;

byte noLinkChar[8] = {
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10001,
  0b00000,
  0b11111, 
  0b00000
};

// |||----------------------------|||-КЛАСС КНОПОК (GND)-|||----------------------------|||
class Button {
  private:
    uint8_t pin;
    unsigned long lastChange;
    bool state;
    bool lastReading;
    bool stateChanged;

  public:
    Button(uint8_t p) : pin(p), lastChange(0), state(false), lastReading(false), stateChanged(false) {}
    
    void begin() { pinMode(pin, INPUT_PULLUP); }
    
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
  
  // Инициализация RS485
  pinMode(PIN_RS485_EN, OUTPUT);
  digitalWrite(PIN_RS485_EN, LOW); // По умолчанию СЛУШАЕМ
  rs485.begin(9600);
  
  swMode.begin();
  swNet.begin();
  btnScreen.begin();
  btnAction.begin();
  
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, noLinkChar); 
  
  lcd.setCursor(0, 0);
  lcd.print(F("BALER CONTROL"));
  lcd.setCursor(0, 1);
  lcd.print(F("SYSTEM START..."));
  delay(1000);
  lcd.clear();
}

// |||----------------------------|||-ФУНКЦИИ СВЯЗИ RS485-|||----------------------------|||
void pollSlave() {
  // Очищаем мусор из буфера
  while (rs485.available()) rs485.read();

  digitalWrite(PIN_RS485_EN, HIGH); // ПЕРЕДАЧА
  delay(2);
  
  rs485.write(0xBB); // Стартовый байт Мастера
  
  uint8_t crc = 0;
  uint8_t* ptr = (uint8_t*)&txData;
  for (uint16_t i = 0; i < sizeof(MasterData); i++) {
    rs485.write(ptr[i]);
    crc ^= ptr[i];
  }
  rs485.write(crc);

  rs485.flush(); // Ждем окончания физической отправки байтов
  digitalWrite(PIN_RS485_EN, LOW); // ПРИЕМ

  // Ждем ответ от пресса (максимум 100 мс)
  unsigned long waitStart = millis();
  bool replied = false;
  
  while (millis() - waitStart < 100) {
    if (rs485.available() > 0) {
      if (rs485.read() == 0xAA) { // Нашли стартовый байт Пресса
        delay(10); // Ждем пока весь пакет долетит
        
        if (rs485.available() >= sizeof(SlaveData) + 1) {
          uint8_t rCrc = 0;
          uint8_t* rPtr = (uint8_t*)&slaveData;
          
          for (uint16_t i = 0; i < sizeof(SlaveData); i++) {
            rPtr[i] = rs485.read();
            rCrc ^= rPtr[i];
          }
          
          // Проверяем контрольную сумму
          if (rCrc == rs485.read()) {
            replied = true;
            txData.doReset = false; // Команда сброса успешно доставлена, снимаем флаг
          }
        }
        break; // Выходим из цикла ожидания
      }
    }
  }
  
  isConnected = replied; // Обновляем статус связи для экрана
}

void loop() {
  swMode.update();
  swNet.update();
  btnScreen.update();
  btnAction.update();

  // --- ОБЩЕНИЕ ПО RS485 (каждые 300 мс) ---
  if (millis() - lastPollTime >= 300) {
    pollSlave();
    lastPollTime = millis();
  }

  // --- ЛОГИКА ЭКРАНА (Короткое нажатие) ---
  if (btnScreen.justPressed()) {
    screenPage = (screenPage == 0) ? 1 : 0; 
    screenTimer = millis();                 
    lcd.clear();                            
  }

  // --- ЛОГИКА ДЕЙСТВИЯ ---
  if (swMode.isPressed()) {
    // В ручном режиме мотор включается физически
  } else {
    // В АВТО режиме работает удержание 1 сек для сброса
    if (btnAction.isHeldFor(1000) && !resetCommandSent) {
      Serial.println(F("SEND RS485: RESET COMMAND!"));
      txData.doReset = true; // Записываем команду в пакет (отправится при следующем pollSlave)
      resetCommandSent = true; 
      
      screenPage = 0; 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F(">> SBROS OK <<  "));
      delay(1000); 
      lcd.clear();
    }
    if (!btnAction.isPressed()) {
      resetCommandSent = false;
    }
  }

  if (screenPage == 1 && (millis() - screenTimer >= 5000)) {
    screenPage = 0;
    lcd.clear();
  }

  if (millis() - lastDisplayUpdate > 200) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

// |||----------------------------|||-ИНТЕРФЕЙС ЭКРАНА-|||----------------------------|||
void updateDisplay() {
  if (screenPage == 0) {
    lcd.setCursor(0, 0);
    
    if (swMode.isPressed()) { 
      if (btnAction.isPressed()) lcd.print(F("PUSK MOTORA!   "));
      else lcd.print(F("GOTOV K RABOTE "));
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
    if (!isConnected) lcd.write(0); 
    else lcd.print(F(" ")); 

    lcd.setCursor(0, 1);
    
    char buffer[17];
    char modesStr[9];
    strcpy(modesStr, swMode.isPressed() ? "[R]" : "[A]");
    strcat(modesStr, swNet.isPressed() ? "[SET]" : "[SHP]");
    
    sprintf(buffer, "%-8s%8u", modesStr, slaveData.sessionBales);
    lcd.print(buffer);
  } 
  
  else if (screenPage == 1) {
    lcd.setCursor(0, 0);
    lcd.print(F("TOTAL RULONOV:  ")); 
    
    lcd.setCursor(15, 0);
    if (!isConnected) lcd.write(0);
    
    lcd.setCursor(0, 1);
    char buffer[17];
    sprintf(buffer, "      %-10u", slaveData.totalBales);
    lcd.print(buffer);
  }
}
