// 31 mars 2022 : Demo code for 2 MFX locs, 1 Turn (adr=5) & S88 (16 inputs)
// MFX est une marque dÃ©posÃ©e par MÃ¤rklin
// Ce code minimaliste a besoin de la Gleisbox pour lire l'UID prÃ©sent dans chaque locomotive
// voir procÃ©dure dans Â§4.1 de http://gelit.ch/Train/Raildue_F.pdf


/*
BTS7960 High Current 43A H-Bridge Motor Driver
https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf
*/

#include "RPi_Pico_TimerInterrupt.h"

// Pin D0-D1 used by USB
const uint8_t signalPinHigh = 2;  // voir Â§10 de http://gelit.ch/Train/DirectMM2.pdf
const uint8_t signalPinLow = 3;
const uint8_t powerPin = 8;   // pin 1 L293 (enable H-Bridge)
const uint8_t redLedPin = 9;  // Power Status

// s88
// const int clockPin = 10;  // S88  voir Â§5.3 de http://gelit.ch/Train/Raildue_F.pdf
// const int dataPin = 11;
// const int resetPin = 12;
// const int loadPin = 13;

//const int s88_Nb = 16;         // 1 module Littfinski RM-88-N-Opto
const uint8_t maxLocomotives = 2;  // Taille du tableau des locomotives
const uint8_t Rob = 1;             // Robel 39548
const uint8_t BLS = 2;             // BLS 29486

// Toutes ces variables sont globales et presentes en PERMANENCE en RAM
volatile byte signalBuffer[100];     // MFX Buffer (type BYTE to store Length)
volatile bool isBusy;                // many MFX Writers (centrale, periodic, getSID, Speed, ...) --> only 1 at the same time (isBusy=0 by Interrupt)
volatile int bitIndex;               // MM2_Bit
volatile uint16_t crcValue;          // Krauss p13
volatile bool isFirstBit;            // MFX First bit always 1
volatile byte interruptBufferIndex;  // Interrupt buffer index signalBuffer[i]);
volatile bool signalLevel;           // MFX Level (polarite)
volatile byte frameLength;           // MFX Frame  Length
volatile byte activeLocomotive;      // only 1 Loc is active for Speed & Direction
volatile byte locomotiveRefresh;     // Refresh
volatile byte timerCounter;      // MFX Interrupt
volatile byte syncCount;             // 3 caract de synchro MFX au min
//volatile bool physicalRegister[s88_Nb + 10]; // S88 Physical Register
//volatile byte mm2Packet[18 + 4];  // MM2 Packet for Turn
volatile bool isPowerOn;
volatile byte systemState;      // State in loop
volatile int stateMachineStep;  // MFX State Machine
volatile byte bitStuffing;      // Stuffing bit
volatile bool toSend;
volatile bool trace;    // Debug
volatile bool turnCmd;  // Turn Cmd
volatile bool turnVal;  // Turn Value
volatile byte turnAdr;  // Turn Adress
volatile unsigned long tempoCentrale, tempoPeriodic, T_S88, Time;
volatile byte frameCounter;  // Zahler in centrale Frame

// Structure des locomotives
struct Locomotive {
  bool direction;  // Direction
  byte speed;      // Vitesse
  bool light;      // Lumière
  byte UID[4];
};
volatile struct Locomotive locomotive[maxLocomotives + 2];

bool timerHandler(struct repeating_timer *t);
void changeSignalLevel();
void throttle(byte, byte);
void adr(byte loc);
void func(byte, byte, bool);
void periodic(byte);
void adr0();
void centrale();
void calculateCRC();
//void s88();
//void Turn(int dev, bool val);
void bCRC(bool b);
void tri(int v, int b);
void getSID(byte);

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);


void setup() {
  trace = true;  // debug
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(100);

  Serial.println("DirectMFX");

  gpio_init(powerPin);
  gpio_set_dir(powerPin, GPIO_OUT);
  gpio_put(powerPin, LOW);
  gpio_init(redLedPin);
  gpio_set_dir(redLedPin, GPIO_OUT);
  gpio_put(redLedPin, LOW);

  isPowerOn = false;

  gpio_init(signalPinHigh);
  gpio_set_dir(signalPinHigh, GPIO_OUT);
  gpio_init(signalPinLow);
  gpio_set_dir(signalPinLow, GPIO_OUT);

  locomotive[Rob].UID[0] = 0x73;
  locomotive[Rob].UID[1] = 0xfb;
  locomotive[Rob].UID[2] = 0x17;
  locomotive[Rob].UID[3] = 0x40;
  locomotive[BLS].UID[0] = 0xF9;
  locomotive[BLS].UID[1] = 0xF2;
  locomotive[BLS].UID[2] = 0x3A;
  locomotive[BLS].UID[3] = 0xC1;

  activeLocomotive = 1;
  Serial.println("Robel is active");
  turnAdr = 5;
  turnVal = false;

  // s88
  // pinMode(dataPin, INPUT);
  // pinMode(clockPin, OUTPUT);
  // pinMode(loadPin, OUTPUT);
  // pinMode(resetPin, OUTPUT);

  Serial.print(F("\nStarting TimerInterrupt on "));
  Serial.println(BOARD_NAME);
  Serial.println(RPI_PICO_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));

  if (ITimer0.attachInterruptInterval(50, timerHandler))  // interrupt every 50 us (micro seconde)
    Serial.printf("[Main %d] Starting ITimer0 OK, millis() =  %d\n", __LINE__, millis());
  else {
    Serial.printf("[Main %d] Can't set ITimer0. Select another freq. or timer\n", __LINE__);
    return;
  }
  Serial.flush();

  isBusy = false;
  systemState = 1;
  stateMachineStep = 1;
  timerCounter = 0;
  syncCount = 0;
  toSend = false;
  bitStuffing = 0;
  turnCmd = false;
  signalLevel = HIGH;  // Â§2.2.9 Pause mit neg Potential
  Serial.println("Power OFF - Enter to Power ON");
}  // end setup

void loop() {
  byte cmd;
  switch (systemState) {
    case 0:  // stay here after PowerOFF
      break;
    case 1:
      if (Serial.available() > 0) {
        cmd = Serial.read();  // start here from setup

        //if (cmd == 13) {
        if (cmd == 10) {
          Serial.println("Power ON");
          stateMachineStep = 10;
          delay(200);
          systemState = 2;
        }
      }  // send Sync & PowerON
      break;
    case 2:
      if (isPowerOn) {
        delay(500);
        digitalWrite(redLedPin, HIGH);
        Serial.println("PowerON (Enter to PowerOFF)");
        systemState = 3;
      }
      break;
    case 3:
      frameCounter = 1;
      centrale();
      delay(500);
      centrale();
      delay(500);
      centrale();
      delay(500);
      systemState = 4;  //  Â§4.2
      break;
    case 4:
      getSID(Rob);
      delay(200);
      frameCounter++;
      centrale();
      delay(200);
      Serial.println("getSID Robel");
      getSID(BLS);
      delay(200);
      frameCounter++;
      centrale();
      delay(200);
      Serial.println("getSID BLS");
      locomotive[Rob].speed = 0;
      throttle(Rob, locomotive[Rob].speed);
      locomotive[Rob].direction = 0;
      locomotive[Rob].light = 1;
      func(Rob, 0, locomotive[Rob].light);
      locomotive[BLS].speed = 0;
      throttle(BLS, locomotive[BLS].speed);
      locomotive[BLS].direction = 0;
      locomotive[BLS].light = 1;
      func(BLS, 0, locomotive[BLS].light);
      locomotiveRefresh = 1;
      tempoPeriodic = millis() + 50;
      tempoCentrale = millis() + 500;
      //T_S88 = millis() + 1000;  // set Timing
      systemState = 5;
      break;
    case 5:
      // if (millis() > T_S88)
      // {
      //   T_S88 = millis() + 1000;
      // s88();
      // }
      if (millis() > tempoCentrale) {
        tempoCentrale = millis() + 500;
        centrale();
      }

      // Envoi periodique des infos des differentes locos sur la voie
      if (millis() > tempoPeriodic) {
        tempoPeriodic = millis() + 50;
        periodic(locomotiveRefresh);
        locomotiveRefresh++;
        if (locomotiveRefresh == maxLocomotives + 1) {
          locomotiveRefresh = 1;
        }
      }  // Find Next Decoder

      if (Serial.available() > 0) {
        cmd = Serial.read();
        if (cmd > 64 && cmd < 91) {
          Serial.println("CapsLock !!!");
        }
        if (cmd >= '0' && cmd <= '7') {
          locomotive[activeLocomotive].speed = cmd - 48;
          throttle(activeLocomotive, locomotive[activeLocomotive].speed);
          Serial.print("Speed = ");
          Serial.println(locomotive[activeLocomotive].speed);
        }
        switch (cmd) {
          case '8':
            activeLocomotive = Rob;
            Serial.println("Robel");
            break;
          case '9':
            activeLocomotive = BLS;
            Serial.println("BLS");
            break;
          case 'l':
            if (locomotive[activeLocomotive].light) {
              locomotive[activeLocomotive].light = 0;
            } else {
              locomotive[activeLocomotive].light = 1;
            }
            func(activeLocomotive, 0, locomotive[activeLocomotive].light);
            Serial.println("Toggle Light");
            break;
          case 'd':
            if (locomotive[activeLocomotive].direction) {
              locomotive[activeLocomotive].direction = 0;
            } else {
              locomotive[activeLocomotive].direction = 1;
            }
            throttle(activeLocomotive, 0);
            Serial.println("Toggle Direction");
            break;
          // case 't':
          //   turnVal = !turnVal;
          //   Turn(turnAdr, turnVal);
          //   delay(250);
          //   Serial.print("turnVal=");
          //   Serial.println(turnVal);
          //   break;
          case 's':
            Serial.printf("Power = %s \nSystemState = %d \nMachine steep = %d\n", isPowerOn ? "true" : "false", systemState, stateMachineStep);
            Serial.printf("Length = %d\n", signalBuffer[0]);
            Serial.print("CRC = 0x");
            Serial.println(crcValue, HEX);
            Serial.print("SignalBuffer : ");
            for (byte i = 1; i <= signalBuffer[0]; i++)
              Serial.printf(" %d", signalBuffer[i]);

            Serial.println();
            // Serial.print("MM2 = ");
            // for (byte i = 0; i <= 18; i++) {
            //   Serial.print(mm2Packet[i]);
            //   Serial.print(" ");
            // }
            // Serial.println();
            // Serial.print("S88  ");
            // for (a = 1; a <= 16; a++)
            // {
            //   Serial.print(a);
            //   Serial.print("=");
            //   Serial.print(physicalRegister[a]);
            //   Serial.print(" ");
            // }
            // Serial.println();
            break;
          case 10:
            //case 13:
            if (isPowerOn) {
              systemState = 1;
              stateMachineStep = 1;
              digitalWrite(powerPin, 0);
              isPowerOn = false;
              digitalWrite(redLedPin, 0);
              Serial.println("Power OFF");
            }
            break;
          case 'h':
            Serial.println("Speed:0-7  Robel:8  BLS:9  l:Light  d:Direction  t:Turn  s:Statistics");
            break;
        }
      }
      break;
  }
}  // end loop

void adr0() {
  signalBuffer[1] = 1;
  for (byte i = 2; i < 10; i++) {
    signalBuffer[i] = 0;
  }
  frameLength += 9;  // frameLength incrémenté pour 1 + 8 bits à 0
}  // always 7 bit adr (signalBuffer[0] used for LENGTH)

void adr(byte loc) {  // set SID
  signalBuffer[1] = 1;
  signalBuffer[2] = 0;  // always 7 bit adr (signalBuffer[0] used for LENGTH)
  for (byte i = 0, b = 6; i < 8; i++, b--)
    signalBuffer[i + 3] = loc & (1 << b);
  frameLength = 9;
}

void throttle(byte loc, byte value) {  // Â§3.1.2 Fahren mit 8 steps
  if (trace) {
    Serial.print("Speed-");
    Serial.print(loc);
    Serial.print("-");
    Serial.println(value);
  }
  // do {
  // } 
  while (isBusy){};
  isBusy = true;
  frameLength = 0;
  adr(loc);
  signalBuffer[10] = 0;
  signalBuffer[11] = 0;
  signalBuffer[12] = 0;
  frameLength = frameLength + 3;
  if (locomotive[loc].direction) {
    signalBuffer[13] = 1;
  } else {
    signalBuffer[13] = 0;
  }
  frameLength = frameLength + 1;  // Direction
  // byte b = 2;
  // for (byte i = 0; i < 3; i++) {
  //   signalBuffer[i + 14] = bitRead(value, b);
  //   b--;
  // }
  for (byte i = 0, b = 2; i < 3; i++, b--)
    signalBuffer[i + 14] = value & (1 << b);

  frameLength = frameLength + 3;
  signalBuffer[0] = frameLength;  // Length in FIRST BYTE
  calculateCRC();
  toSend = true;
}

void func(byte loc, byte func, bool value) {  // Â§3.1.6 Einzel Funktion
  if (trace) {
    Serial.print("func-");
    Serial.print(loc);
    Serial.print("-");
    Serial.print(func);
    Serial.print("-");
    Serial.println(value);
  }
  // do {
  // } 
  while (isBusy){};
  isBusy = true;
  frameLength = 0;
  adr(loc);
  signalBuffer[10] = 1;
  signalBuffer[11] = 0;
  signalBuffer[12] = 0;
  frameLength = frameLength + 3;
  byte b = 6;
  for (byte i = 0; i < 8; i++) {
    signalBuffer[i + 13] = bitRead(func, b);
    b--;
  }
  frameLength = frameLength + 7;
  signalBuffer[20] = 0;
  signalBuffer[21] = value;
  frameLength = frameLength + 2;
  signalBuffer[0] = frameLength;  // Length in FIRST BYTE
  calculateCRC();
  toSend = true;
}

void periodic(byte loc) {  // like MS2
  if (trace) {
    Serial.print("periodic-");
    Serial.println(loc);
  }
  // do {
  // } 
  while (isBusy){};
  isBusy = true;
  frameLength = 0;
  adr(loc);
  signalBuffer[10] = 0;
  signalBuffer[11] = 0;
  signalBuffer[12] = 1;
  frameLength += 3;
  signalBuffer[13] = (locomotive[loc].direction) ? 1 : 0;
  frameLength += 1;

  for (byte i = 0, b = 2; i < 3; i++, b--) {
    signalBuffer[i + 14] = locomotive[loc].speed & (1 << b);
    frameLength++;
  }

  signalBuffer[17] = 0;
  signalBuffer[18] = 0;
  signalBuffer[19] = 0;
  signalBuffer[20] = 0;
  frameLength += 4;  // only MSB
  signalBuffer[21] = 0;
  signalBuffer[22] = 1;
  signalBuffer[23] = 1;
  signalBuffer[24] = 1;
  frameLength += 4;  // Â§3.1.5 F15-F0
  signalBuffer[25] = 0;
  signalBuffer[26] = 0;
  signalBuffer[27] = 0;
  signalBuffer[28] = 0;
  signalBuffer[29] = 0;
  signalBuffer[30] = 0;
  signalBuffer[31] = 0;
  signalBuffer[32] = 0;
  frameLength += 8;
  signalBuffer[33] = 0;
  signalBuffer[34] = 0;
  signalBuffer[35] = 0;
  signalBuffer[36] = 0;
  signalBuffer[37] = 0;
  signalBuffer[38] = 0;
  signalBuffer[39] = 0;
  signalBuffer[40] = locomotive[loc].light;
  frameLength += 8;
  signalBuffer[0] = frameLength;  // Length in FIRST BYTE
  calculateCRC();
  toSend = true;
}

void getSID(byte loc) {  // fixed SID from UID
  if (trace) {
    Serial.print("getSID-");
    Serial.println(loc);
  }
  // do {
  // } 
  while (isBusy){};
  isBusy = true;
  frameLength = 0;
  adr0();
  signalBuffer[10] = 1;
  signalBuffer[11] = 1;
  signalBuffer[12] = 1;
  signalBuffer[13] = 0;
  signalBuffer[14] = 1;
  signalBuffer[15] = 1;
  frameLength = frameLength + 6;  // Â§3.2.4  111 011 AAAAAAAAAAAAAA UID
  signalBuffer[16] = 0;
  signalBuffer[17] = 0;
  signalBuffer[18] = 0;
  signalBuffer[19] = 0;
  signalBuffer[20] = 0;
  signalBuffer[21] = 0;
  signalBuffer[22] = 0;
  frameLength = frameLength + 7;

  for (byte i = 0, b = 6; i < 7; i++, b--) {
    signalBuffer[i + 23] = loc & (1 << b);
    frameLength++;
  }

  for (byte j = 0; j < 4; j++) {
    for (byte i = 0, count = 30, b = 7; i < 8; i++, count++, b--) {
      signalBuffer[count] = locomotive[loc].UID[j] & (1 << b);
      frameLength++;
    }
  }

  signalBuffer[0] = frameLength;  // Length in FIRST BYTE
  calculateCRC();
  toSend = true;
}

void centrale() {  // p23
  if (trace) {
    Serial.println("Centrale");
  }
  // do {
  // } 
  while (isBusy){};
  isBusy = true;
  frameLength = 0;
  adr0();

  byte value[] = {
    1, 1, 1, 1, 0, 1,
    0, 1, 0, 0, 0, 1, 1, 1,
    0, 1, 1, 0, 1, 0, 1, 1,
    1, 0, 1, 1, 0, 1, 1, 1,
    1, 1, 0, 1, 1, 1, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0
  };

  // Copie les valeurs dans signalBuffer à partir de l'index 10
  for (int i = 0; i < sizeof(value); i++) {
    signalBuffer[10 + i] = value[i];
    frameLength++;
  }

  for (byte i = 0, b = 7; i < 8; i++, b--) {
    signalBuffer[i + 56] = frameCounter & (1 << b);
    frameLength++;
  }

  signalBuffer[0] = frameLength;  // Length in FIRST BYTE
  calculateCRC();
  toSend = true;
}

bool timerHandler(struct repeating_timer *t) {
  (void)t;
  timerCounter++;  // Important !
  switch (stateMachineStep) {
    case 1:
      break;  // stay here
    case 3:
      switch (timerCounter) {  // Â§2.2.9 "mindestens 2 Sync"
        case 1:
        case 3:
        case 4:
        case 6:
        case 8:
        case 9:
          changeSignalLevel();
          break;
        case 10:
          syncCount++;
          timerCounter = 0;
          if (syncCount > 1) {
            stateMachineStep = 4;
          }
          break;
      }
      break;

    case 4:
      if (timerCounter == 125) {
        bitIndex = 0;
        stateMachineStep = 5;  // Â§2.2.9 Pause=6.25 ms --> 6250/50=125
        ITimer0.setInterval(104, timerHandler);
      }
      break;  // Interrupt = 104 us

    case 5:  // Send First Turn Packet
      // if (mm2Packet[bitIndex]) {
      //   changeSignalLevel();
      //   delayMicroseconds(85);
      //   changeSignalLevel();
      // }  // HIGH : 85 --> Scope=91
      // else {
      //   changeSignalLevel();
      //   delayMicroseconds(9);
      //   changeSignalLevel();
      // }  // LOW  :  9           13
      bitIndex++;
      if (bitIndex == 18) {
        timerCounter = 0;
        stateMachineStep = 6;
      }
      break;

    case 6:
      if (timerCounter == 15) {
        bitIndex = 0;
        stateMachineStep = 7;
      }
      break;  // Â§2.2.9 Pause=1.5 ms

    case 7:  // Send Second Turn Packet
      // if (mm2Packet[bitIndex]) {
      //   changeSignalLevel();
      //   delayMicroseconds(85);
      //   changeSignalLevel();
      // } else {
      //   changeSignalLevel();
      //   delayMicroseconds(9);
      //   changeSignalLevel();
      // }
      bitIndex++;
      if (bitIndex == 18) {
        timerCounter = 0;
        stateMachineStep = 8;
        ITimer0.setInterval(50, timerHandler);
      }  // Interrupt = 50 us
      break;

    case 8:
      if (timerCounter == 124) {  // Â§2.2.9 Pause=6.18 ms --> 6180/50=124
        turnCmd = false;
        timerCounter = 0;
        syncCount = 0;
        stateMachineStep = 10;
      }
      break;

    case 10:
      switch (timerCounter) {  // Synchro
        case 1:
          changeSignalLevel();
          if (!isPowerOn && systemState != 0) {
            digitalWrite(powerPin, 1);
            isPowerOn = true;
            digitalWrite(redLedPin, 1);
          }
          break;
        case 3:
        case 4:
        case 6:
        case 8:
        case 9:
          changeSignalLevel();
          break;

        case 10:
          if (toSend && syncCount > 3) {
            toSend = false;
            syncCount = 0;
            bitStuffing = 0;
            isFirstBit = 1;
            interruptBufferIndex = 1;
            frameLength = signalBuffer[0];
            stateMachineStep = 20;
          } else if (turnCmd) {
            timerCounter = 0;
            syncCount = 0;
            stateMachineStep = 3;
          } else {
            timerCounter = 0;
            syncCount++;
          }
          break;
      }
      break;

    case 20:
      if (isFirstBit)
        changeSignalLevel();  // Data stream with CRC
      else {
        if (signalBuffer[interruptBufferIndex]) {
          changeSignalLevel();
          bitStuffing++;
          // après 8 bits à un (1) transmis, un zéro (0) est toujours inséré
          if (bitStuffing == 8) {
            bitStuffing = 0;
            isFirstBit = false;
            stateMachineStep = 30;
          }
        }  // isFirstBit=0; + isFirstBit=!isFirstBit; --> isFirstBit=1
        else
          bitStuffing = 0;

        interruptBufferIndex++;
        frameLength--;
        if (frameLength == 0) {
          stateMachineStep = 10;
          timerCounter = 0;
          isBusy = false;
        }  // NEW
      }
      isFirstBit = !isFirstBit;
      break;

    case 30:
      if (isFirstBit)
        changeSignalLevel();  // Send "0" after 8 "1"
      else
        stateMachineStep = 20;
      isFirstBit = !isFirstBit;
      break;
  }
  return true;
}

void changeSignalLevel() {
  if (signalLevel) {
    gpio_put(signalPinHigh, LOW);  // Mettre à LOW la broche haute
    gpio_put(signalPinLow, HIGH);   // Mettre à HIGH la broche basse
  } else {
    gpio_put(signalPinHigh, HIGH);  // Mettre à HIGH la broche haute
    gpio_put(signalPinLow, LOW);   // Mettre à LOW la broche basse
  }
  signalLevel = !signalLevel;  // Inverser l'état du signal
}

void calculateCRC() {  // avoid to compute in interrupt ! --> easier for bit stuffing !
  crcValue = 0x007F;
  for (byte i = 1; i < signalBuffer[0] + 1; i++) {
    bCRC(signalBuffer[i]);
  }  // CRC
  for (byte i = 0; i < 8; i++) {
    bCRC(0);
  }  // Krauss p13 "diese bit mÃ¼ssen zuerst mit 0 belegt ..."}


  for (byte i = 0, b = 7; i < 8; i++, b--) {
    signalBuffer[frameLength + 1 + i] = crcValue & (1 << b);
    signalBuffer[0]++;
  }
}

void bCRC(bool b) {  // Krauss p13
  crcValue = (crcValue << 1) + b;
  if ((crcValue & 0x0100) > 0)
    crcValue = (crcValue & 0x00FF) ^ 0x07;
}
