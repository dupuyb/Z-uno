#include <EEPROM.h>

/*
  Version Z-Uno 1.8.4.app
  Tools Borad Z-Wave>ME Z-Uno
  Fequency Europe
  Multicommand Disabled
  Programmer: Z-uno programmer
*/
// Debuger mode
#define DEBUG

#define LED_PIN       13 // pin 13 User LED of Z-Uno board  /!\ Connrcted to PMW1

// buttons definition
#define BTN_PIN_UP    18 // pin 18 Button UP Manual button open valve
#define BTN_PIN_DN    17 // pin 17 Button Dn Manual button close valve

// RGB definition
#define RED_PIN       PWM2 // pin 21 connection Bleu
#define GREEN_PIN     PWM3 // pin 22 connection Rouge
#define BLUE_PIN      PWM4 // pin 23 connection Vert

// I/O definition
#define REL_PIN       9  // pin 16 relay open or close valve
#define BUZ_PIN      10  // pin 10 buzzer ring during valve moving

// Flux detector
#define ANA_PIN_FX    A0 // pin 10 Sonor Flux sensor
// #define DIG_PIN_FX    19 // pin 19 Sonor Flux Digital

// EEPROM
#define EEPROM_ADDR    0x800    // Addr where EEprom data is stored

// variable to store last values
int8_t command;                 // Serial command
uint8_t lastButtonUp;           // Manual button open valve
uint8_t lastButtonDn;           // Manual button close valve
uint8_t pushboth;               // Up and Down are pressed a same time
int16_t nbrLoop;                // Counter of number of mean for acquisition
int16_t crtFluxValue;           // Crt Aqn 10 bits [from 0 to 1023]
int32_t maxFluxValue;           // Max Aqn 10 bits [from 0 to 1023]
int16_t lastFluxValue;          // Last 10 bits [from 0 to 1023]
boolean isValveClosed = false;  // Valve must be colsed or open
uint16_t stateValveInSec = 0;   // Seconds valve since the valve is closed
uint16_t alarmDelay = 0;        // Seconds where the buzzer is ringing

// Timer simulation
uint32_t previousMillis = 0; // Last milli-second
uint8_t seconds = 0;         // Seconds [0-59]
uint8_t minutes = 0;         // Minutes [0-59]
uint8_t heures  = 0;         // Hours
uint8_t cumule  = 0;         // Numbre of seconds with fulx detected

struct eprom_data {
  uint32_t ticksWater;   // init at zero Number of cumulatted minutes
  uint16_t lastFluxRef;  // Init at 160 on 10 bits set at 50% from [05 to 100%]
  uint8_t  lastDelayMin; // Init at 60 secondes [from 1 to 99 minutes]
  bool     valveClose;   // True if valve is clsed
  uint8_t  crc8;
};
eprom_data    epData;            // Persistant data in EEprom

// Channels declaration. This code reports the following to the Z-Wave controller:
ZUNO_SETUP_CHANNELS(
// - Binary valve state O=Close
                    ZUNO_SWITCH_BINARY(getterValveSt, setterValveSt),
// - Binary WarningTimer if WarningTimer > DelayTimer then close the valve
                    ZUNO_SWITCH_BINARY(getterValveWd, setterValveWd),
// - Dimmer Reference for Flux detection if sensor > Reference then Water is streaming
                    ZUNO_SWITCH_MULTILEVEL(getterFluxRef, setterFluxRef),
// - Dimmer DelayTimer form 0 to 99 seconds
                    ZUNO_SWITCH_MULTILEVEL(getterDelayMin, setterDelayMin),
// - Loudness Current Water sensor value
                    ZUNO_SENSOR_MULTILEVEL(ZUNO_SENSOR_MULTILEVEL_TYPE_LOUDNESS,
                                           SENSOR_MULTILEVEL_SCALE_PERCENTAGE_VALUE,
                                           SENSOR_MULTILEVEL_SIZE_TWO_BYTES,
                                           SENSOR_MULTILEVEL_PRECISION_ONE_DECIMAL,
                                           getterFluxVal),
// - Meter Ticks minutes total water stream
                    ZUNO_METER(ZUNO_METER_TYPE_WATER,
                               METER_RESET_ENABLE,
                               ZUNO_METER_WATER_SCALE_PULSECOUNT,
                               METER_SIZE_FOUR_BYTES,
                               METER_PRECISION_ZERO_DECIMALS,
                               getterTicksWater, resetterTicksWater)

);

ZUNO_SETUP_ASSOCIATIONS(ZUNO_ASSOCIATION_GROUP_SET_VALUE, ZUNO_ASSOCIATION_GROUP_SET_VALUE_AND_DIM);

// the setup routine runs once when you press reset:
void setup() {
  command = 0;
#ifdef DEBUG
  Serial.begin(115200);
#endif
  previousMillis = millis();
  // Set I/O Directions
  pinMode(REL_PIN,    OUTPUT);
  pinMode(BUZ_PIN,    OUTPUT);
  pinMode(BLUE_PIN,   OUTPUT);
  pinMode(RED_PIN,    OUTPUT);
  pinMode(GREEN_PIN,  OUTPUT);
  pinMode(LED_PIN,    OUTPUT); // set LED pin as output keyboard
  pinMode(BTN_PIN_UP, INPUT_PULLUP); // set button pin as input
  pinMode(BTN_PIN_DN, INPUT_PULLUP); // set button pin as input
  // pinMode(DIG_PIN_FX, INPUT); // No used
  // White during startup
  setRGB(255, 255, 255);

  // Get last values from EEPROM
  EEPROM.get(EEPROM_ADDR,  &epData, sizeof(eprom_data));
  // Check data
  if (myCrc8((uint8_t*)&epData, sizeof(eprom_data) - 1) != epData.crc8) {
    // Invalid data - reset all
    epData.ticksWater   = 0;    // init at zero
    epData.lastFluxRef  = 160;  // Init at 160 on 10 bits set at 50% from [05 to 100%]
    epData.lastDelayMin = 60;   // Init at 60 secondes [from 1 to 99 minutes]
    epData.valveClose = false;  // Valve is Open
    updateEprom();
  } else {
    // Restore previous state
    isValveClosed = epData.valveClose;
  }
  maxFluxValue = 0;
  // Relay switched off on reset and dischage off
  // digitalWrite(REL_PIN, LOW); Set by isValveClosed
  digitalWrite(BUZ_PIN, LOW);
}

// the loop routine runs over and over again forever:
void loop() {
#ifdef DEBUG
  while (Serial.available() > 0) {
    uint8_t c = (uint8_t)Serial.read();
    if (c != 13) {
      command = c;
    } else {
      if (command == 'd') {
        Serial.println("Command in debug mode...");
      } else {
        if (command == 'p') Serial.println("Command in plotter mode...");
        else Serial.println("Command stop serial.");
      }
    }
  }
#endif

  boolean bothpushed = false;
  // read all buttons
  uint8_t crtButtonUp  = digitalRead(BTN_PIN_UP);
  uint8_t crtButtonDn  = digitalRead(BTN_PIN_DN);
  // Set action if button pushed
  lastButtonUp = getButton(crtButtonUp , lastButtonUp);
  lastButtonDn = getButton(crtButtonDn , lastButtonDn);
  if (lastButtonUp == 0 && lastButtonDn == 0) {
    bothpushed = true;
  } else {
    pushboth = 0;
    if (lastButtonUp == 0) {
      if (isValveClosed == true) {
        isValveClosed = false; // Open valse and wait action completed
        stateValveInSec = 0;
      }
    } else {
      if (lastButtonDn == 0) {
        if (isValveClosed == false) {
          isValveClosed = true; // Close valve
          stateValveInSec = 0;
        }
      }
    }
  }

  // Listen Flux in pipe
  crtFluxValue = analogRead(ANA_PIN_FX);
  // Get Max flux add all and compute mean
  // if (crtFluxValue > maxFluxValue)
  maxFluxValue += crtFluxValue;
  nbrLoop++; // There is betwwen 1000 and 1100 loop per secondes

  // Main loop every new second elapes
  if ( millis() - previousMillis > 1000L) {
    previousMillis = millis();

    // Get Max Flux in pipe tube
    lastFluxValue = (int16_t) (maxFluxValue/nbrLoop);
    maxFluxValue = 0;

    // Change level reference (low hysteresis)
    uint16_t fluxRefAdj = epData.lastFluxRef;
    if (stateValveInSec > 0 && fluxRefAdj > 3) {
      fluxRefAdj = fluxRefAdj - 3;
    }

    // if valve is open do evaluation
    if (isValveClosed == false) {
      if (lastFluxValue > fluxRefAdj) {
        cumule ++;
        if (cumule>=59) {
          epData.ticksWater++;
          cumule = 0;
        }
        if (stateValveInSec == 0) {
          myzunoSendReport(2);
        }
        if ( (stateValveInSec / 60) < epData.lastDelayMin) {
          // Every 4 seconds send getterValveWd
          if ((stateValveInSec & 0x3) == 0x3) myzunoSendReport(2);
          stateValveInSec++;
        } else {
          // Close valve
          isValveClosed = true;
        }
      } else {
        // Valve is closed
        if (stateValveInSec != 0) {
          stateValveInSec = 0;
          myzunoSendReport(2);
        }
      }
    }

    // Every 30 seconds send getterFluxVal (30 seconds for refresh max for dimmer)
    if ( seconds == 5 || seconds == 35) {
      myzunoSendReport(5);
    }

#ifdef DEBUG
    if (command == 'd') {
      float pec = lastFluxValue * (99.0 / 1023.0);
      float ref = epData.lastFluxRef * (99.0 / 1023.0);
      printHMS();
      Serial.print(" Fx:"); printFMT(pec, 2);
      Serial.print("% Rf:"); printFMT(ref, 2);
      Serial.print("% Wd:"); printIMT(epData.lastDelayMin * 60, 4);
      Serial.print("s nAcq:"); printIMT(nbrLoop, 4);
      Serial.print("p/sec Tot:"); printIMT(epData.ticksWater, 5);
      Serial.print("mts Fx>Rf:"); printIMT(stateValveInSec, 3);
      Serial.println("p");
    }
    if (command == 'p') {
      float pec = lastFluxValue * (99.0 / 1023.0);
      float ref = epData.lastFluxRef * (99.0 / 1023.0);
      Serial.print(pec); Serial.print(" "); Serial.print(ref); Serial.print(" ");
      Serial.println(stateValveInSec);
    }
#endif

    // Reset number of loop
    nbrLoop = 0;

    // Both touch pressed
    if (bothpushed) {
      setRGB(0, 0, 255); // bleu
      isValveClosed = false; // Open valse and wait action completed
      stateValveInSec = 0;
      if (pushboth++ == 4) {
        // Reset total meter
        setRGB(255, 255, 255); // bleu
        resetterTicksWater(0);
        pushboth = 0;
        myzunoSendReport(6);
      }
    } else {
      // Led color
      if (isValveClosed) {
        setRGB(255, 0, 0); // red
      } else {
        if (stateValveInSec > 0)
          setRGB(255, 160, 0); // orange
        else
          setRGB(0, 255, 0);  // green
      }
    }

    // Action valve And notify
    if (actionValve()) {
      alarmDelay = 3;
      myzunoSendReport(1);
      myzunoSendReport(2);
    }

    if (alarmDelay>0) alarmDelay--;

    // Timer incrementation
    if (seconds++ == 59) {
      seconds = 0;
      if (myCrc8((byte*)&epData, sizeof(eprom_data) - 1) != epData.crc8) {
        updateEprom();
        myzunoSendReport(6);
      }
      if (minutes++ == 59) {
        minutes = 0;
        heures++;
      }
    }
  }
}

// ---------------------- Functions ----------------------------
void myzunoSendReport(byte c) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("myzunoSendReport:" ); Serial.println(c, DEC); }
#endif
  zunoSendReport(c);
}

uint8_t myCrc8(uint8_t * data, uint8_t count) {
  uint8_t result = 0xDF;
  while (count--) {
    result ^= *data;
    data++;
  }
  return result;
}
void updateEprom() {
  epData.crc8 = myCrc8((uint8_t*)&epData, sizeof(eprom_data) - 1);
  EEPROM.put(EEPROM_ADDR, &epData, sizeof(eprom_data));
#ifdef DEBUG
  if (command == 'd') { Serial.print("updateEprom crc:" ); Serial.println(epData.crc8, DEC); }
#endif
}

void printIMT(uint32_t fl, uint8_t nd) {
  if (nd>=5 && fl<10000) Serial.print("0");
  if (nd>=4 && fl<1000) Serial.print("0");
  if (nd>=3 && fl<100) Serial.print("0");
  if (nd>=2 && fl<10) Serial.print("0");
  Serial.print(fl);
}

void printFMT(float fl, uint8_t nd) {
  if (nd>=3 && fl<100) Serial.print("0");
  if (nd>=2 && fl<10) Serial.print("0");
  Serial.print(fl);
}

void printHMS() {
  if (heures < 10) Serial.print("0");
  Serial.print(heures);
  Serial.print(":");
  if (minutes < 10) Serial.print("0");
  Serial.print(minutes);
  Serial.print(":");
  if (seconds < 10) Serial.print("0");
  Serial.print(seconds);
}

boolean  actionValve() {
  if (isValveClosed) {
    digitalWrite (REL_PIN, HIGH);
  } else {
    digitalWrite(REL_PIN, LOW);
  }
  boolean ret = (epData.valveClose != isValveClosed) ;
  epData.valveClose = isValveClosed;
  return ret;
}

// Button action
uint8_t getButton(uint8_t crt, uint8_t last) {
  if (crt != last) {
    if (crt == LOW) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  }
  return crt;
}

// RGB setup
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  if ( (seconds & 0x1) == 0 ) {
    uint8_t s = ((command == 'd' || command == 'p') ? (5) : (0));
    analogWrite(RED_PIN,   s);
    analogWrite(GREEN_PIN, s);
    analogWrite(BLUE_PIN,  s);
  } else {
    analogWrite(RED_PIN,   r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN,  b);
  }
  if ( alarmDelay > 0 ) {
    if ( (seconds & 0x1) == 0 )
      digitalWrite(BUZ_PIN, HIGH);
    else
      digitalWrite(BUZ_PIN, LOW);
  } else {
    digitalWrite(BUZ_PIN, LOW);
  }
}

// ------------------- Delay WatchDog-- --------------------------
uint8_t getterDelayMin(void) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("getterDelayMin(4):" ); Serial.println(epData.lastDelayMin, DEC); }
#endif
  return epData.lastDelayMin;
}

void setterDelayMin(uint8_t value) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("setterDelayMin:" ); Serial.println(value, DEC); }
#endif
  if (value == 0) value = 1;
  if (value > 99) value = 99;
  epData.lastDelayMin = value;
  updateEprom();
}
//----------------------- Valve State & Wd -----------------------------
// 0 for closed 1-99 for open
uint8_t getterValveSt(void) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("getterValveSt(1):" ); Serial.println(isValveClosed); }
#endif
  return ((isValveClosed) ? 0x0 : 0xFF);
}

uint8_t getterValveWd(void) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("getterValveWd(2):" ); Serial.println(stateValveInSec); }
#endif
  return ((stateValveInSec == 0) ? 0xFF : 0x0);
}

// 0 for closed 1-99 for open
void setterValveSt(uint8_t value) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("setterValveSt:" ); Serial.println(value); }
#endif
  if (value > 0) {
    isValveClosed = false;
    stateValveInSec = 0;
  } else {
    isValveClosed = true;
  }
}

//-
void setterValveWd(uint8_t value) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("setterValveWd:" ); Serial.println(value, DEC); }
#endif
  stateValveInSec = 0; // Reset WD & open valve
  isValveClosed = false;
}
//----------------------- Flux value ------------------------------------

int16_t getterFluxVal(void) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("getterFluxVal(5):" ); Serial.println(lastFluxValue); }
#endif
  return lastFluxValue;
}

//----------------------- Meter Water ------------------------------------
uint32_t getterTicksWater(void) {
#ifdef DEBUG
  if (command == 'd') { Serial.print("getterTicksWater(6):" ); Serial.println(epData.ticksWater); }
#endif
  return epData.ticksWater;
}

void resetterTicksWater(byte v) { // Jamais appelé avec JEEDOM !!!
  // Reset compteur
  epData.ticksWater = v * 0;
  updateEprom();
#ifdef DEBUG
  if (command=='d') { Serial.println("resetterTicksWater at zero" ); }
#endif
}
//------------------------ Ref Sensor --------------------------------------
void setterFluxRef(uint8_t value) {
#ifdef DEBUG
  if (command == 'd') {
    Serial.print("setterFluxRef:" ); Serial.println(value, DEC);
  }
#endif
  if (value > 99) value = 99;
  epData.lastFluxRef = (int16_t)((float)value * (1023.0 / 99.0));
  updateEprom();
}

uint8_t getterFluxRef(void) {
  float fl = (epData.lastFluxRef * (99.0 / 1023.0));
  uint8_t ret = (uint8_t)fl;
#ifdef DEBUG
  if (command == 'd') {
    Serial.print("getterFluxRef(3):(" ); Serial.print(ret, DEC); Serial.print(")=" ); Serial.println(epData.lastFluxRef, DEC);
  }
#endif
  return ret;
}
