#include <SPI.h>
#include <WiFi101.h>

// =============================================================================
// Configuration Constants
// =============================================================================

// WiFi Settings
#define WIFI_SSID       "Group 19"
#define WIFI_PASSWORD   "ShutDown1"

// Pin Definitions - Microcontroller
#define PIN_WIFI_CS     8
#define PIN_WIFI_IRQ    7
#define PIN_WIFI_RST    4
#define PIN_WIFI_EN     2

// Pin Definitions - IR Sensors
#define PIN_IR_LEFT_ANALOG   A1
#define PIN_IR_LEFT_DIGITAL  A0
#define PIN_IR_RIGHT_ANALOG  A3
#define PIN_IR_RIGHT_DIGITAL A2

// Pin Definitions - Ultrasonic Sensor
#define PIN_UR_TRIG     0
#define PIN_UR_ECHO     1

// Pin Definitions - Motors
#define PIN_FL_PWM      6
#define PIN_FL_DIR      5
#define PIN_FR_PWM      11
#define PIN_FR_DIR      12
#define PIN_BL_PWM      A4
#define PIN_BL_DIR      A5
#define PIN_BR_PWM      9
#define PIN_BR_DIR      10

// Motor forward polarity
#define FL_FORWARD_LOGIC  false
#define FR_FORWARD_LOGIC  true
#define BL_FORWARD_LOGIC  false
#define BR_FORWARD_LOGIC  true

// Emergency Stop Settings
#define EMERGENCY_STOP_DISTANCE   25    // cm - stop when obstacle closer than this
#define EMERGENCY_RESUME_DISTANCE 40    // cm - resume when obstacle farther than this

// PID Settings
#define DEFAULT_BASE_SPEED    80
#define DEFAULT_MIN_SPEED     -120
#define DEFAULT_MAX_SPEED     120
#define DEFAULT_ALPHA         0.3
#define DEFAULT_KP            100.0
#define DEFAULT_KI            0.05
#define DEFAULT_KD            0.0
#define DEFAULT_MAX_INTEGRAL  200.0

// Timing Settings
#define STREAM_INTERVAL_MS    500
#define SERIAL_BAUD_RATE      115200
#define PULSE_TIMEOUT_US      30000

// =============================================================================
// Classes
// =============================================================================

class Microcontroller {
private:
  int csPin;
  int irqPin;
  int rstPin;
  int enPin;

  const char* ssid;
  const char* password;

  WiFiServer server;

public:
  Microcontroller(int cs, int irq, int rst, int en, const char* apSSID, const char* apPass)
    : csPin(cs), irqPin(irq), rstPin(rst), enPin(en), ssid(apSSID), password(apPass), server(80) 
  {}

  void begin() {
    pinMode(enPin, OUTPUT);
    pinMode(rstPin, OUTPUT);
    pinMode(irqPin, INPUT);
    pinMode(csPin, OUTPUT);
  }

  void enableWiFi() {
    digitalWrite(enPin, HIGH);
    delay(100);

    WiFi.setPins(csPin, irqPin, rstPin, enPin);

    IPAddress localIP(192, 168, 4, 1);
    WiFi.config(localIP);

    int status = WiFi.beginAP(ssid, password);
    if (status != WL_AP_LISTENING) {
      Serial.print("Failed to start AP, status=");
      Serial.println(status);
      return;
    }

    Serial.print("AP started: ");
    Serial.println(ssid);

    server.begin();
  }

  WiFiServer& getServer() {
    return server;
  }
};

class IrSensor {
private:
  int analogPin;
  int digitalPin;

public:
  IrSensor(int aPin, int dPin) : analogPin(aPin), digitalPin(dPin) {}

  void begin() {
    pinMode(analogPin, INPUT);
    pinMode(digitalPin, INPUT);
  }

  int readAnalog() {
    return analogRead(analogPin);
  }

  int readDigital() {
    return digitalRead(digitalPin);
  }
};

class UrSensor {
private:
  int trigPin;
  int echoPin;

public:
  UrSensor(int trig, int echo) : trigPin(trig), echoPin(echo) {}

  void begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  long getDistanceCM() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
    if (duration == 0) return 999;  // Timeout - no obstacle detected

    return duration / 58;
  }
};

class Motor {
private:
  int pwmPin;
  int dirPin;
  bool forwardLogic;
  int speed;
  bool forward;

public:
  Motor(int pwm, int dir, bool fwdLogic) 
    : pwmPin(pwm), dirPin(dir), forwardLogic(fwdLogic), speed(0), forward(true) {}

  void begin() {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }

  void set(int newSpeed, bool newForward) {
    speed = constrain(newSpeed, 0, 255);
    forward = newForward;

    bool dirSignal = (forward == forwardLogic);
    digitalWrite(dirPin, dirSignal ? HIGH : LOW);
    analogWrite(pwmPin, speed);
  }

  void stop() {
    speed = 0;
    analogWrite(pwmPin, 0);
  }

  int getSpeed() {
    return speed;
  }

  bool isForward() {
    return forward;
  }

  int getSignedSpeed() {
    if (speed == 0) return 0;
    return forward ? speed : -speed;
  }
};

// Forward declaration
void resetPID();

class Robot {
private:
  IrSensor &irLeft, &irRight;
  UrSensor &ur;
  Motor &FL, &FR, &BL, &BR;
  Microcontroller &mcu;

  bool inPID = false;
  bool emergencyActivated = true;
  bool emergencyStopped = false;
  
  int motorSpeed = DEFAULT_BASE_SPEED;

public:
  Robot(
    IrSensor &irL,
    IrSensor &irR,
    UrSensor &u,
    Motor &fl,
    Motor &fr,
    Motor &bl,
    Motor &br,
    Microcontroller &mc
  ) :
    irLeft(irL),
    irRight(irR),
    ur(u),
    FL(fl),
    FR(fr),
    BL(bl),
    BR(br),
    mcu(mc)
  {}

  void begin() {
    irLeft.begin();
    irRight.begin();
    ur.begin();
    FL.begin();
    FR.begin();
    BL.begin();
    BR.begin();
    mcu.begin();
  }

  // -------------------------------------------------------------------------
  // Motor Control
  // -------------------------------------------------------------------------
  
  void stopAll() {
    FL.stop();
    FR.stop();
    BL.stop();
    BR.stop();
  }

  void setLeftMotors(int speed) {
    FL.set(abs(speed), speed >= 0);
    BL.set(abs(speed), speed >= 0);
  }

  void setRightMotors(int speed) {
    FR.set(abs(speed), speed >= 0);
    BR.set(abs(speed), speed >= 0);
  }

  void setMotorSpeed(int speed) {
    motorSpeed = constrain(speed, 0, 255);
  }

  int getMotorSpeed() {
    return motorSpeed;
  }

  // -------------------------------------------------------------------------
  // Movement Functions
  // -------------------------------------------------------------------------

  void moveNorth() {
    FL.set(motorSpeed, true);
    FR.set(motorSpeed, true);
    BL.set(motorSpeed, true);
    BR.set(motorSpeed, true);
  }

  void moveSouth() {
    FL.set(motorSpeed, false);
    FR.set(motorSpeed, false);
    BL.set(motorSpeed, false);
    BR.set(motorSpeed, false);
  }

  void moveEast() {
    FL.set(motorSpeed, true);
    FR.set(motorSpeed, false);
    BL.set(motorSpeed, false);
    BR.set(motorSpeed, true);
  }

  void moveWest() {
    FL.set(motorSpeed, false);
    FR.set(motorSpeed, true);
    BL.set(motorSpeed, true);
    BR.set(motorSpeed, false);
  }

  void moveNorthEast() {
    FL.set(motorSpeed, true);
    FR.set(0, true);
    BL.set(0, true);
    BR.set(motorSpeed, true);
  }

  void moveNorthWest() {
    FL.set(0, true);
    FR.set(motorSpeed, true);
    BL.set(motorSpeed, true);
    BR.set(0, true);
  }

  void moveSouthEast() {
    FL.set(0, true);
    FR.set(motorSpeed, false);
    BL.set(motorSpeed, false);
    BR.set(0, true);
  }

  void moveSouthWest() {
    FL.set(motorSpeed, false);
    FR.set(0, true);
    BL.set(0, true);
    BR.set(motorSpeed, false);
  }

  void rotateClockwise() {
    FL.set(motorSpeed, true);
    FR.set(motorSpeed, false);
    BL.set(motorSpeed, true);
    BR.set(motorSpeed, false);
  }

  void rotateCounterClockwise() {
    FL.set(motorSpeed, false);
    FR.set(motorSpeed, true);
    BL.set(motorSpeed, false);
    BR.set(motorSpeed, true);
  }

  // -------------------------------------------------------------------------
  // Sensor Access
  // -------------------------------------------------------------------------

  int getLeftIR() { 
    return irLeft.readAnalog(); 
  }
  
  int getRightIR() { 
    return irRight.readAnalog(); 
  }

  long getDistance() {
    return ur.getDistanceCM();
  }

  // -------------------------------------------------------------------------
  // Emergency Stop
  // -------------------------------------------------------------------------

  void setEmergencyActivated(bool activated) {
    emergencyActivated = activated;
    if (!activated) {
      emergencyStopped = false;
    }
  }

  bool isEmergencyActivated() {
    return emergencyActivated;
  }

  bool isEmergencyStopped() {
    return emergencyStopped;
  }

  // Returns true if robot should stop due to emergency
  bool checkEmergencyStop() {
    if (!emergencyActivated) return false;

    long distance = ur.getDistanceCM();

    if (distance < EMERGENCY_STOP_DISTANCE) {
      emergencyStopped = true;
    } else if (distance > EMERGENCY_RESUME_DISTANCE) {
      emergencyStopped = false;
    }

    if (emergencyStopped) {
      stopAll();
      return true;
    }
    return false;
  }

  // -------------------------------------------------------------------------
  // PID Control
  // -------------------------------------------------------------------------

  void startPID() {
    stopAll();
    resetPID();
    inPID = true;
  }

  void stopPID() {
    inPID = false;
    stopAll();
    resetPID();
  }

  bool isInPID() {
    return inPID;
  }

  // -------------------------------------------------------------------------
  // Diagnostics
  // -------------------------------------------------------------------------

  void printInformation(Print &out) {
    out.println("=== Sensor Readings ===");
    out.print("IR Left (Analog): ");
    out.println(irLeft.readAnalog());
    out.print("IR Left (Digital): ");
    out.println(irLeft.readDigital());
    out.print("IR Right (Analog): ");
    out.println(irRight.readAnalog());
    out.print("IR Right (Digital): ");
    out.println(irRight.readDigital());
    out.print("Ultrasonic Distance (cm): ");
    out.println(ur.getDistanceCM());

    out.println();
    out.println("=== Motor Speeds (signed) ===");
    out.print("Front Left: ");
    out.println(FL.getSignedSpeed());
    out.print("Front Right: ");
    out.println(FR.getSignedSpeed());
    out.print("Back Left: ");
    out.println(BL.getSignedSpeed());
    out.print("Back Right: ");
    out.println(BR.getSignedSpeed());

    out.println();
    out.println("=== Status ===");
    out.print("PID Mode: ");
    out.println(inPID ? "ON" : "OFF");
    out.print("Emergency Stop Active: ");
    out.println(emergencyActivated ? "YES" : "NO");
    out.print("Emergency Stopped: ");
    out.println(emergencyStopped ? "YES" : "NO");
  }
};

class RequestHandler {
private:
  Robot &robot;

public:
  RequestHandler(Robot &r) : robot(r) {}

  void handle(WiFiClient &client) {
    if (!client || !client.connected()) return;

    String request = client.readStringUntil('\n');
    while (client.available()) client.read();

    String response = "OK";

    // PID Control
    if (request.indexOf("GET /pid/on") >= 0) {
      robot.startPID();
      response = "PID started";
    } 
    else if (request.indexOf("GET /pid/off") >= 0) {
      robot.stopPID();
      response = "PID stopped";
    }
    // Emergency Stop Control
    else if (request.indexOf("GET /emergency/on") >= 0) {
      robot.setEmergencyActivated(true);
      response = "Emergency stop enabled";
    }
    else if (request.indexOf("GET /emergency/off") >= 0) {
      robot.setEmergencyActivated(false);
      response = "Emergency stop disabled";
    }
    // Movement Commands
    else if (request.indexOf("GET /move/north") >= 0) {
      robot.moveNorth();
      response = "Moving North";
    }
    else if (request.indexOf("GET /move/south") >= 0) {
      robot.moveSouth();
      response = "Moving South";
    }
    else if (request.indexOf("GET /move/east") >= 0) {
      robot.moveEast();
      response = "Moving East";
    }
    else if (request.indexOf("GET /move/west") >= 0) {
      robot.moveWest();
      response = "Moving West";
    }
    else if (request.indexOf("GET /move/northeast") >= 0) {
      robot.moveNorthEast();
      response = "Moving NorthEast";
    }
    else if (request.indexOf("GET /move/northwest") >= 0) {
      robot.moveNorthWest();
      response = "Moving NorthWest";
    }
    else if (request.indexOf("GET /move/southeast") >= 0) {
      robot.moveSouthEast();
      response = "Moving SouthEast";
    }
    else if (request.indexOf("GET /move/southwest") >= 0) {
      robot.moveSouthWest();
      response = "Moving SouthWest";
    }
    else if (request.indexOf("GET /move/cw") >= 0) {
      robot.rotateClockwise();
      response = "Rotating Clockwise";
    }
    else if (request.indexOf("GET /move/ccw") >= 0) {
      robot.rotateCounterClockwise();
      response = "Rotating Counter-Clockwise";
    }
    else if (request.indexOf("GET /stop") >= 0) {
      robot.stopAll();
      response = "Stopped";
    }
    // Speed Control
    else if (request.indexOf("GET /speed/") >= 0) {
      int speedStart = request.indexOf("/speed/") + 7;
      int speedEnd = request.indexOf(" ", speedStart);
      if (speedEnd == -1) speedEnd = request.length();
      String speedStr = request.substring(speedStart, speedEnd);
      int newSpeed = speedStr.toInt();
      robot.setMotorSpeed(newSpeed);
      response = "Speed set to " + String(newSpeed);
    }

    // Send HTTP Response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println(response);
    client.println();
    client.print("PID: ");
    client.println(robot.isInPID() ? "ON" : "OFF");
    client.print("Emergency: ");
    client.println(robot.isEmergencyActivated() ? "ON" : "OFF");
  }
};

// =============================================================================
// Global Objects
// =============================================================================

Microcontroller mcu(PIN_WIFI_CS, PIN_WIFI_IRQ, PIN_WIFI_RST, PIN_WIFI_EN, WIFI_SSID, WIFI_PASSWORD);

IrSensor irLeft(PIN_IR_LEFT_ANALOG, PIN_IR_LEFT_DIGITAL);
IrSensor irRight(PIN_IR_RIGHT_ANALOG, PIN_IR_RIGHT_DIGITAL);

UrSensor ur(PIN_UR_TRIG, PIN_UR_ECHO);

Motor FL(PIN_FL_PWM, PIN_FL_DIR, FL_FORWARD_LOGIC);
Motor FR(PIN_FR_PWM, PIN_FR_DIR, FR_FORWARD_LOGIC);
Motor BL(PIN_BL_PWM, PIN_BL_DIR, BL_FORWARD_LOGIC);
Motor BR(PIN_BR_PWM, PIN_BR_DIR, BR_FORWARD_LOGIC);

Robot robot(irLeft, irRight, ur, FL, FR, BL, BR, mcu);

RequestHandler requestHandler(robot);

WiFiClient streamClient;
WiFiClient controlClient;
unsigned long lastSend = 0;

// =============================================================================
// PID Variables
// =============================================================================

unsigned long lastMillis = 0;
double e = 0.0;
double ef = 0.0;
double integralSum = 0.0;
double previousError = 0.0;

int baseSpeed   = DEFAULT_BASE_SPEED;
int minSpeed    = DEFAULT_MIN_SPEED;
int maxSpeed    = DEFAULT_MAX_SPEED;
int leftSpeed   = 0;
int rightSpeed  = 0;

double alpha        = DEFAULT_ALPHA;
double kP           = DEFAULT_KP;
double kI           = DEFAULT_KI;
double kD           = DEFAULT_KD;
double correction   = 0.0;
double maxIntegral  = DEFAULT_MAX_INTEGRAL;

bool pidFirstRun = true;

// =============================================================================
// Functions
// =============================================================================

void resetPID() {
  lastMillis = millis();
  e = 0.0;
  ef = 0.0;
  integralSum = 0.0;
  previousError = 0.0;
  leftSpeed = 0;
  rightSpeed = 0;
  correction = 0.0;
  pidFirstRun = true;
}

void pidControl() {
  if (!robot.isInPID()) return;

  // Check emergency stop first
  if (robot.checkEmergencyStop()) {
    return;  // Robot stopped due to obstacle
  }

  unsigned long now = millis();
  
  // Skip first iteration to establish baseline
  if (pidFirstRun) {
    lastMillis = now;
    pidFirstRun = false;
    return;
  }

  double dt = (now - lastMillis) / 1000.0;
  if (dt <= 0.001) return;  // Skip if time delta too small

  int leftSensor  = constrain(robot.getLeftIR(), 0, 1023);
  int rightSensor = constrain(robot.getRightIR(), 0, 1023);

  // Calculate error
  e = double(leftSensor) - double(rightSensor);
  
  // Apply low-pass filter
  ef = alpha * e + (1.0 - alpha) * ef;

  // Integral term with anti-windup
  integralSum += ef * dt;
  double maxIsum = maxIntegral / max(fabs(kI), 1e-9);
  integralSum = constrain(integralSum, -maxIsum, maxIsum);

  // Derivative term with clamping to prevent spikes
  double derivative = (ef - previousError) / dt;
  derivative = constrain(derivative, -1000.0, 1000.0);  // Clamp derivative

  // Calculate correction
  correction = kP * ef + kI * integralSum + kD * derivative;

  // Apply to motors
  leftSpeed  = constrain(round(baseSpeed - correction), minSpeed, maxSpeed);
  rightSpeed = constrain(round(baseSpeed + correction), minSpeed, maxSpeed);

  robot.setLeftMotors(leftSpeed);
  robot.setRightMotors(rightSpeed);

  previousError = ef;
  lastMillis = now;
}

void handleClients() {
  // Accept new clients
  WiFiClient newClient = mcu.getServer().available();
  
  if (newClient) {
    if (!streamClient || !streamClient.connected()) {
      streamClient = newClient;
    } else if (!controlClient || !controlClient.connected()) {
      controlClient = newClient;
    } else {
      // Both slots full, use as control client
      controlClient = newClient;
    }
  }

  // Stream sensor data periodically
  if (streamClient && streamClient.connected() && millis() - lastSend >= STREAM_INTERVAL_MS) {
    robot.printInformation(streamClient);
    streamClient.println("----------------------");
    streamClient.flush();
    lastSend = millis();
  }

  // Handle control requests
  if (controlClient && controlClient.connected() && controlClient.available()) {
    requestHandler.handle(controlClient);
    delay(5);
    controlClient.stop();
  }
}

// =============================================================================
// Arduino Entry Points
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(100);  // Give serial time to initialize
  
  Serial.println("Self-Driving Bot Starting...");
  
  // Initialize all hardware
  robot.begin();
  
  // Enable WiFi
  mcu.enableWiFi();
  
  // Initialize PID timing
  lastMillis = millis();
  
  Serial.println("Setup complete!");
}

void loop() {
  handleClients();
  pidControl();
}
