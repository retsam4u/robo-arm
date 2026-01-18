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
  bool updated;
};

struct ServoCommand {
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
void checkBluetooth(RobotArm &arm);
bool readBtServoCommand(SoftwareSerial &bt, ServoCommand &outCmd);
void sendRoboArmDataToBluetooth(SoftwareSerial &bt, RobotArm &arm);
void checkIR(RobotArm &arm);
void debugIR();
bool readIrServoCommand(IRrecv &irReceiver, ServoCommand cmd);
ServoCommand mapIrCommandToServoCommand(uint16_t irCommand);
void handleServoCommand(RobotArm &arm, ServoCommand cmd);
void checkAndApplyRobotArmState(RobotArm &arm, SoftwareSerial &bt);
void setupServoMotors();
void setServoAngles(RobotArm &arm);

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
  checkBluetooth(robotArm);
  checkIR(robotArm);
  checkAndApplyRobotArmState(robotArm, hc05);
  delay(15);
}

// ===================================================================
//   BLUETOOTH CONTROL
// ===================================================================
void checkBluetooth(RobotArm &arm) {
  // ------- CONTROL BLUETOOTH -------
  ServoCommand cmd;
  if (readBtServoCommand(hc05, cmd)) {
    handleServoCommand(arm, cmd);
  }
}

// Read from stream until we have a complete command like: >...<
// Return true only when we have extracted a complete and valid command.
bool readBtServoCommand(SoftwareSerial &bt, ServoCommand &outCmd) {
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

void sendRoboArmDataToBluetooth(SoftwareSerial &bt, RobotArm &arm) {
  if (bt.availableForWrite() > 0) {
    String data = "";
    for (int servoIndex = 0; servoIndex < 6; servoIndex++) {
      data = data + (servoIndex + 1) + "," + arm.angle[servoIndex] + (servoIndex == 5 ? "" : ";");
    }
    bt.print(data);
  }
}

// ===================================================================
//   IR CONTROL
// ===================================================================
void checkIR(RobotArm &arm) {
    // ------- CONTROL IR -------
  if (IrReceiver.decode()) {
    // debugID();
    ServoCommand cmd;
    if (readIrServoCommand(IrReceiver, cmd)) {
      handleServoCommand(arm, cmd);
    }
  }
}

void debugIR() {
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
}

bool readIrServoCommand(IRrecv &irReceiver, ServoCommand cmd) {
    /*
      * Check the received data and perform actions according to the received command
      */
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
        Serial.println(F("Repeat received. Here you can repeat the same action as before."));
        return false;

    } else {
        Serial.print("IR command = ");
        Serial.println(IrReceiver.decodedIRData.command);
        cmd = ServoCommand{0, "", 0}; // mapIrCommandToServoCommand(IrReceiver.decodedIRData.command);
        return cmd.servo > 0;
    }
}

ServoCommand mapIrCommandToServoCommand(uint16_t irCommand) {
  switch (irCommand) {
    case 0x10: return ServoCommand{1, "-", 10};
    case 0x11: return ServoCommand{1, "+", 10};
    default: return ServoCommand{0, "", 0};
  }
}

// ===================================================================
//   General function for command processing
// ===================================================================
void handleServoCommand(RobotArm &arm, ServoCommand cmd) {
  uint8_t servoIndex = cmd.servo - 1;

  int angle = arm.angle[servoIndex];

  if (cmd.op == '=') angle = cmd.value;
  else if (cmd.op == '+') angle = angle + cmd.value;
  else if (cmd.op == '-') angle = angle - cmd.value;

  angle = clampAngle(angle);

  arm.angle[servoIndex] = angle;
  arm.updated = true;
}

void checkAndApplyRobotArmState(RobotArm &arm, SoftwareSerial &bt) {
  if (arm.updated) {
    setServoAngles(arm);
    arm.updated = false;
    sendRoboArmDataToBluetooth(bt, arm);
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
  robotArm.updated = false;
}

void setServoAngles(RobotArm &arm) {
  for (int i = 0; i < 6; i++) {
    servos[i].write(arm.angle[i]);
  }
}
