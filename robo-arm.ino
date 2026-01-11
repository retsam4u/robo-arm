/********************************************************************
  ROBOTIC ARM
  - uses Arduino Nano
  - with 6 servomotors
  - with control from IR and bluetooth (HC-05)
  - arduino code structured with OOP

  (c)2026 by Valentin Protiuc
********************************************************************/
#include <Arduino.h>
#include <IRremote.hpp>
#include <SoftwareSerial.h>
#include <Servo.h>

// -------- PIN CONFIG --------
#define IR_RECEIVER_PIN  2     // IR Receiver
#define RED_LED_PIN      3     // Red LED - command confirmation
#define GREEN_LED_PIN    12    // Green LED - running program confirmation

#define SERVO_PINS {4, 5, 6, 7, 8, 9}  // Pins for the 6 servomotors

#define BT_RX_PIN        10    // RX Arduino (connects to TXD HC-05)
#define BT_TX_PIN        11    // TX Arduino (connects to RXD HC-05)

// ===================================================================
//   STRUCT
// ===================================================================
struct RobotArm {
  int angle[6];  // angle of each servo
};

struct BtCommand {
  uint8_t servo;  // 1..6
  char op;        // '+', '-', '='
  int value;      // 0..180 (or step)
};

// ===================================================================
//   VARIABLES
// ===================================================================
SoftwareSerial hc05(BT_RX_PIN, BT_TX_PIN);
RobotArm robotArm;
Servo servos[6];

// ===================================================================
//   OOP Functions
// ===================================================================
void setupServoMotors();
void setServoAngles(RobotArm &arm);
void handleCommand(RobotArm &arm, int command);
void controlWithIR(RobotArm &arm, unsigned long irCode);
void controlWithBluetooth(RobotArm &arm, char btChar);

// ===================================================================
//   STATIC FUNCTIONS
// ===================================================================
static inline int clampAngle(int a) {
  if (a < 0) return 0;
  if (a > 180) return 180;
  return a;
}



// ===================================================================
//   SETUP
// ===================================================================
void setup() {
  // LEDs setup
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  // Serial initialization
  Serial.begin(9600);     // Serial standard

  // IR setup
  pinMode(IR_RECEIVER_PIN, INPUT);
  IrReceiver.begin(IR_RECEIVER_PIN, true, RED_LED_PIN);

  // Bluetooth setup
  hc05.begin(9600);    // Bluetooth HC-05 (Nano supports Serial1)

  // Servo Setup
  setupServoMotors();

    // Green LED ON -> confirmation of active program
  digitalWrite(GREEN_LED_PIN, HIGH);
  Serial.println("STARTED");

  IrReceiver.printIRResultMinimal(&Serial);
}

// ===================================================================
//   LOOP
// ===================================================================
void loop() {
  checkBluetooth();
  checkIR();
  delay(15);
}

// ===================================================================
//   Functions to check in loop
// ===================================================================
void checkBluetooth() {
  // ------- CONTROL BLUETOOTH -------
  BtCommand cmd;
  if (readBtCommand(hc05, cmd)) {
    uint8_t i = cmd.servo - 1;

    if (cmd.op == '=') angle[i] = cmd.value;
    else if (cmd.op == '+') angle[i] = angle[i] + cmd.value;
    else if (cmd.op == '-') angle[i] = angle[i] - cmd.value;

    angle[i] = clampAngle(angle[i]);
    Serial.print("angle[");
    Serial.print(i);
    Serial.print("]=");
    Serial.println(angle[i]);
  }
}

void checkIR() {
    // ------- CONTROL IR -------
  if (IrReceiver.decode()) {
    /*
      * Print a summary of received data
      */
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
        Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
        // We have an unknown protocol here, print extended info
        IrReceiver.printIRResultRawFormatted(&Serial, true);

        IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
    } else {
        IrReceiver.resume(); // Early enable receiving of the next IR frame

        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.printIRSendUsage(&Serial);
    }
    Serial.println();

    /*
      * Finally, check the received data and perform actions according to the received command
      */
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
        Serial.println(F("Repeat received. Here you can repeat the same action as before."));
    } else {
        Serial.println("command!");
        if (IrReceiver.decodedIRData.command == 0x10) {
            Serial.println(F("Received command 0x10."));
            // do something
        } else if (IrReceiver.decodedIRData.command == 0x11) {
            Serial.println(F("Received command 0x11."));
            // do something else
        }
    }
  }
}

// ===================================================================
//   General function for command processing
// ===================================================================
void handleCommand(RobotArm &arm, int cmd) {
  switch (cmd) {
    case 10: // Servo 1: RESET
      amr.angle[1] = 90;
      break;
    case 11: // Servo 1: -10deg
      arm.angle[1] -= 10;
      break;
    case 12: // Servo 1: -5deg
      arm.angle[1] -= 5;
      break;
    case 13: // Servo 1: +5deg
      arm.angle[1] += 5;
      break;
    case 14: // Servo 1: +10deg
      arm.angle[1] += 10;
      break;
  }

  setServoAngles(arm);
}

// ===================================================================
//   IR CONTROL
// ===================================================================
void controlWithIR(RobotArm &arm, unsigned long irCode) {
  switch (irCode) {

    case 0xFF629D:  // buton 1
      handleCommand(arm, 0);
      break;

    case 0xFF22DD:  // buton 2
      handleCommand(arm, 1);
      break;

    case 0xFF02FD:  // buton 3
      handleCommand(arm, 2);
      break;

    default:
      break;
  }
}

// ===================================================================
//   BLUETOOTH CONTROL
// ===================================================================
void controlWithBluetooth(RobotArm &arm, char btChar) {
  if (btChar >= '0' && btChar <= '5') {
    int index = btChar - '0';
    handleCommand(arm, index);
  }
}

// ===================================================================
//   SERVO FUNCTIONS
// ===================================================================
void setupServoMotors() {
  int pins[] = SERVO_PINS;

  for (int i = 0; i < 6; i++) {
    servos[i].attach(pins[i]);
    robotArm.angle[i] = 90;  // poziție inițială
    servos[i].write(robotArm.angle[i]);
  }
}

void setServoAngles(RobotArm &arm) {
  for (int i = 0; i < 6; i++) {
    servos[i].write(arm.angle[i]);
  }
}

// ===================================================================
//   BLUETOOTH FUNCTIONS
// ===================================================================
// Read from stream until we have a complete command like: >...<
// Return true only when we have extracted a complete and valid command.
bool readBtCommand(SoftwareSerial &bt, BtCommand &outCmd) {
  // Buffer for content between '>' and '<' (without delimitators)
  // Expected format: "4:+10" , "6:=90", "1:-5", "1:=42"
  // Format of readed chars is: >[servoIndex]:[operator][numericValue]<
  // where:
  //    - servoIndex is the index of the changed servo
  //    - operator is one of: '+', '-', '=' (to add, substract or set the numericValue to the angle)
  //    - numericValue is a number between 0 and 180 
  static char buf[16];
  static uint8_t idx = 0;
  static bool inFrame = false;

  while (bt.available() > 0) {
    char c = (char)bt.read();

    if (!inFrame) {
      if (c == '>') {
        inFrame = true;
        idx = 0;
      }
      continue;
    }

    // we are in frame
    if (c == '<') {
      // close frame and keep the received data
      buf[idx] = '\0';
      inFrame = false;

      // Expected: [servoDigit] ':' [op] [number]
      // Example: "4:+10"
      // Simple validation for minimum size of command
      if (idx < 4) return false;

      // 1) servo
      if (buf[0] < '1' || buf[0] > '6') return false;
      uint8_t servo = (uint8_t)(buf[0] - '0');

      // 2) ':'
      if (buf[1] != ':') return false;

      // 3) op
      char op = buf[2];
      if (op != '+' && op != '-' && op != '=') return false;

      // 4) value (the rest of the digits)
      // we only check to be digits
      int val = 0;
      for (uint8_t i = 3; i < idx; i++) {
        if (buf[i] < '0' || buf[i] > '9') return false;
        val = val * 10 + (buf[i] - '0');
        // simple protection against big numbers
        if (val > 10000) return false;
      }

      // validation of the numeric value to be between 0 and 180
      // (for + / - operations this value is a step)
      if (val < 0 || val > 180) return false;

      outCmd.servo = servo;
      outCmd.op = op;
      outCmd.value = val;
      return true;
    }

    // if we get again '>' inside or command then restart (re-sync)
    if (c == '>') {
      idx = 0;
      continue;
    }

    // add char in buffer if we can
    if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      // Buffer overflow -> abandon current frame
      inFrame = false;
      idx = 0;
      return false;
    }
  }

  return false; // we don't have yet a complete command
}
