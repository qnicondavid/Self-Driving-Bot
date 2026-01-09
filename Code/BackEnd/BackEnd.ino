#include <SPI.h>
#include <WiFi101.h>


class Microcontroller {
private:
  int csPin;
  int irqPin;
  int rstPin;
  int enPin;

  const char *ssid;
  const char *password;

  WiFiServer server;

public:
  Microcontroller(int cs, int irq, int rst, int en, const char *apSSID, const char *apPass)
    : csPin(cs), irqPin(irq), rstPin(rst), enPin(en), ssid(apSSID), password(apPass), server(80) {
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

  WiFiServer &getServer() {
    return server;
  }
};

class IrSensor {
private:
  int analogPin;
  int digitalPin;

public:
  IrSensor(int aPin, int dPin) {
    analogPin = aPin;
    digitalPin = dPin;
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
  UrSensor(int trig, int echo) {
    trigPin = trig;
    echoPin = echo;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  long getDistanceCM() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 999;

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
  Motor(int pwm, int dir, bool forwardLogic) {
    pwmPin = pwm;
    dirPin = dir;
    this->forwardLogic = forwardLogic;
    speed = 0;
    forward = true;
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

void resetPID();

unsigned long lastMillis = 0;
double e = 0.0;
double ef = 0.0;
double integralSum = 0.0;
double previousError = 0.0;

int baseSpeed = 120;
int minSpeed = -200;
int maxSpeed = 200;
int leftSpeed = 0;
int rightSpeed = 0;

double alpha = 0.3;
double kP = 200;
double kI = 0;
double kD = 0;
double correction = 0.0;
double maxIntegral = 200.0;

int dStop = 1;
int deltaD = 1;

class Robot {
private:
  IrSensor &irLeft, &irRight;
  UrSensor &ur;

  Motor &FL, &FR, &BL, &BR;

  Microcontroller &mcu;

  bool inPID = false;

  bool emergencyStop = false;

  char telemetryBuf[512];
public:
  Robot(
    IrSensor &irL,
    IrSensor &irR,
    UrSensor &u,
    Motor &fl,
    Motor &fr,
    Motor &bl,
    Motor &br,
    Microcontroller &mc)
    : irLeft(irL),
      irRight(irR),
      ur(u),
      FL(fl),
      FR(fr),
      BL(bl),
      BR(br),
      mcu(mc) {}

  void stopAll() {
    FL.stop();
    FR.stop();
    BL.stop();
    BR.stop();
  }

  void printInformation(Print &out) {
    snprintf(
      telemetryBuf,
      sizeof(telemetryBuf),
      "Sensor readings:\n"
      "\n"
      "IR Left (Analog): %d\n"
      "IR Left (Digital): %d\n"
      "IR Right (Analog): %d\n"
      "IR Right (Digital): %d\n"
      "Ultrasonic Distance (cm): %d\n"
      "\n"
      "EMERGENCY STOP: %s\n"
      "\n"
      "Motor speeds (signed):\n"
      "Front Left: %d\n"
      "Front Right: %d\n"
      "Back Left: %d\n"
      "Back Right: %d\n",
      irLeft.readAnalog(),
      irLeft.readDigital(),
      irRight.readAnalog(),
      irRight.readDigital(),
      getDistance(),
      emergencyStop ? "ACTIVE" : "CLEAR",
      FL.getSignedSpeed(),
      FR.getSignedSpeed(),
      BL.getSignedSpeed(),
      BR.getSignedSpeed());

    out.print(telemetryBuf);
  }


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

  void engageEmergencyStop() {
    if (emergencyStop) return;
    emergencyStop = true;
    emergencyHalt();
  }

  void releaseEmergencyStop() {
    emergencyStop = false;
  }

  void emergencyHalt() {
    stopAll();
  }

  bool isEmergencyStopped() {
    return emergencyStop;
  }

  int getLeftIR() {
    return irLeft.readAnalog();
  }
  int getRightIR() {
    return irRight.readAnalog();
  }

  void setLeftMotors(int speed) {
    FL.set(abs(speed), speed >= 0);
    BL.set(abs(speed), speed >= 0);
  }

  void setRightMotors(int speed) {
    FR.set(abs(speed), speed >= 0);
    BR.set(abs(speed), speed >= 0);
  }

  void move(int speeds[4]) {
    FL.set(abs(speeds[0]), speeds[0] >= 0);
    FR.set(abs(speeds[1]), speeds[1] >= 0);
    BL.set(abs(speeds[2]), speeds[2] >= 0);
    BR.set(abs(speeds[3]), speeds[3] >= 0);
  }
  long getDistance() {
    if (isInPID())
      return 999;
    return ur.getDistanceCM();
  }
};

class RequestHandler {
private:
  Robot &robot;

public:
  RequestHandler(Robot &r)
    : robot(r) {}

  void handle(WiFiClient &client) {
    if (!client || !client.connected()) return;

    String request = client.readStringUntil('\n');

    while (client.available()) client.read();

    if (robot.isEmergencyStopped() && request.indexOf("/move/") >= 0) {
      robot.stopAll();
      return;
    }

    if (request.indexOf("GET /move/north") >= 0) {
      int north[4] = { baseSpeed, baseSpeed, baseSpeed, baseSpeed };
      robot.move(north);
    }

    if (request.indexOf("GET /move/south") >= 0) {
      int south[4] = { -baseSpeed, -baseSpeed, -baseSpeed, -baseSpeed };
      robot.move(south);
    }

    if (request.indexOf("GET /move/east") >= 0) {
      int east[4] = { baseSpeed, -baseSpeed, -baseSpeed, baseSpeed };
      robot.move(east);
    }

    if (request.indexOf("GET /move/west") >= 0) {
      int east[4] = { -baseSpeed, baseSpeed, baseSpeed, -baseSpeed };
      robot.move(east);
    }

    if (request.indexOf("GET /move/cw") >= 0) {
      int cw[4] = { baseSpeed, -baseSpeed, baseSpeed, -baseSpeed };
      robot.move(cw);
    }

    if (request.indexOf("GET /move/ccw") >= 0) {
      int ccw[4] = { -baseSpeed, baseSpeed, -baseSpeed, baseSpeed };
      robot.move(ccw);
    }

    if (request.indexOf("GET /move/nw") >= 0) {
      int nw[4] = { 0, baseSpeed, baseSpeed, 0 };
      robot.move(nw);
    }

    if (request.indexOf("GET /move/ne") >= 0) {
      int ne[4] = { baseSpeed, 0, 0, baseSpeed };
      robot.move(ne);
    }

    if (request.indexOf("GET /move/sw") >= 0) {
      int sw[4] = { -baseSpeed, 0, 0, -baseSpeed };
      robot.move(sw);
    }

    if (request.indexOf("GET /move/se") >= 0) {
      int se[4] = { 0, -baseSpeed, -baseSpeed, 0 };
      robot.move(se);
    }

    if (request.indexOf("GET /move/stop") >= 0) {
      int stopMove[4] = { 0, 0, 0, 0 };
      robot.move(stopMove);
    }

    if (request.indexOf("GET /pid/on") >= 0) {
      if (robot.isEmergencyStopped()) {
        robot.stopPID();
      } else {
        robot.startPID();
      }
    }

    if (request.indexOf("GET /pid/off") >= 0) {
      robot.stopPID();
    }


    int valueIndex = request.indexOf("value=");
    int value = 0;
    if (valueIndex >= 0) {
      String valStr = request.substring(valueIndex + 6);
      int endIdx = valStr.indexOf(' ');
      if (endIdx >= 0) valStr = valStr.substring(0, endIdx);
      value = valStr.toInt();
    }

    if (request.indexOf("GET /pid/kp") >= 0) kP = value;
    if (request.indexOf("GET /pid/ki") >= 0) kI = value;
    if (request.indexOf("GET /pid/kd") >= 0) kD = value;
    if (request.indexOf("GET /pid/base") >= 0) baseSpeed = value;
    if (request.indexOf("GET /pid/speed") >= 0) {
      maxSpeed = abs(value);
      minSpeed = -abs(value);
    }

    if (request.indexOf("GET /emergency/stop") >= 0) dStop = value;
    if (request.indexOf("GET /emergency/delta") >= 0) deltaD = value;

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("PID State:");
    client.println(robot.isInPID() ? "ON" : "OFF");
  }
};

Microcontroller mcu(8, 7, 4, 2, "Group 19", "ShutDown1");

IrSensor irLeft(A1, A0);
IrSensor irRight(A3, A2);

UrSensor ur(0, 1);

Motor FL(6, 5, false);
Motor FR(11, 12, true);
Motor BL(A4, A5, false);
Motor BR(9, 10, true);

Robot robot(irLeft, irRight, ur, FL, FR, BL, BR, mcu);

RequestHandler requestHandler(robot);

WiFiClient streamClient;
WiFiClient controlClient;
unsigned long lastSend = 0;

void printHandler() {
  if (!streamClient || !streamClient.connected()) {
    WiFiClient newClient = mcu.getServer().available();
    if (newClient) streamClient = newClient;
  }

  if (!controlClient || !controlClient.connected()) {
    WiFiClient newClient = mcu.getServer().available();
    if (newClient) controlClient = newClient;
  }

  if (streamClient && streamClient.connected() && millis() - lastSend >= 0) {
    robot.printInformation(streamClient);
    streamClient.println("----------------------");
    streamClient.flush();
    lastSend = millis();
  }

  if (controlClient && controlClient.connected() && controlClient.available()) {
    requestHandler.handle(controlClient);
  }
}

int lastCheck, checkInterval;
bool inMazeSolving;
void setup() {
  mcu.enableWiFi();
  //U Turn
  //Turn(37, 70, -1, 1);
  //Turn(37, 70, -1, -1);

  //
  // Turn(23, 70, -1, 1);
  // Turn(23, 70, -1, 1);
  //threePointTurn();

  //kidnappedA();

  startMaze();
  //baseSpeed = 200;
  //kidnappedB(200, 20, 0);

  //startMaze();

  //timedReverse(3000);
  
  lastCheck = millis();
  checkInterval = 200;
  dStop = 20;
  deltaD = 10;
  //robot.startPID();

}

void loop() {
  //emergencyControl();
  //printHandler();
  //pidControl();
  mazeSolving();
  //emergencyPID();
}

void emergencyPID() {
  unsigned long now = millis();
  if (now - lastCheck > checkInterval) {
    lastCheck = now;
    robot.stopPID();
    if (robot.getDistance() < dStop) {
      robot.stopPID();
      while(1)
        if(robot.getDistance() > dStop + deltaD)
            break;
    }
    robot.startPID();
  }
  pidControl();
}

void timedReverse(int time) {
  int s[4] = { -baseSpeed, -baseSpeed, -baseSpeed, -baseSpeed };
  robot.move(s);
  delay(time);
  robot.stopAll();
}

void kidnappedB(int steps, int time1, int time2) {
  int f[4] = { baseSpeed, baseSpeed, baseSpeed, baseSpeed };
  while (1) {
    bool move = circleMovement(steps, time1, time2);
    if (move)
      break;
    robot.move(f);
    delay(20);
    robot.stopAll();
    delay(1000);
    steps = steps + 100;
    time2 = time2 + 20;
  }
  delay(1000);
  baseSpeed = 120;
  robot.startPID();
  unsigned long start = millis();
  while (1) {
    pidControl();
    if (millis() - start > 10000)
      break;
  }
  robot.stopPID();
}

bool circleMovement(int steps, int time1, int time2) {
  int cw[4] = { baseSpeed, -baseSpeed, baseSpeed, -baseSpeed };
  int e[4] = { baseSpeed, -baseSpeed, -baseSpeed, baseSpeed };
  for (int i = 0; i < steps; i++) {
    robot.stopAll();
    if (detectLine()) return 1;
    robot.move(cw);
    delay(time1);
    robot.stopAll();
    if (detectLine()) return 1;
    robot.move(e);
    delay(time2);
  }
  robot.stopAll();
  return 0;
}


void kidnappedA() {
  int fw[4];
  for (int i = 0; i < 4; i++)
    fw[i] = baseSpeed;
  robot.move(fw);
  while (!detectLine())
    ;
  robot.stopAll();
  delay(1000);
  robot.startPID();
  unsigned long start = millis();
  while (1) {
    pidControl();
    if (millis() - start > 10000)
      break;
  }
  robot.stopPID();
}

bool detectLine() {
  int L = irLeft.readDigital();
  int R = irRight.readDigital();
  return L || R;
}

void startMaze() {
  dStop = 30;
  inMazeSolving = true;
  robot.startPID();
}

void mazeSolving() {
  if (!inMazeSolving)
    return;
  if (inJunction()) {
    robot.stopPID();
    inMazeSolving = false;
    return;
  }
  unsigned long now = millis();
  if (now - lastCheck > checkInterval) {
    lastCheck = now;
    robot.stopPID();
    if (robot.getDistance() < dStop) {
      robot.stopPID();
      recoverDeadEnd();
    }
    robot.startPID();
  }
  pidControl();
}

bool inJunction() {
  int L = irLeft.readDigital();
  int R = irRight.readDigital();
  return L && R;
}

void recoverDeadEnd() {
  int cw[4] = { 120, -120, 120, -120 };
  robot.stopAll();
  robot.move(cw);
  delay(300);
  robot.stopAll();
  while (1) {
    int L = irLeft.readDigital();
    int R = irRight.readDigital();
    if (L || R)
      break;
    robot.move(cw);
    delay(10);
  }
  robot.stopAll();
}

void pidControl() {
  if (robot.isEmergencyStopped()) return;
  if (!robot.isInPID()) return;

  unsigned long now = millis();
  double dt = (now - lastMillis) / 1000.0;
  if (dt <= 0.0) dt = 0.001;

  int leftSensor = constrain(robot.getLeftIR(), 0, 1023);
  int rightSensor = constrain(robot.getRightIR(), 0, 1023);

  e = double(leftSensor) - double(rightSensor);
  ef = alpha * e + (1.0 - alpha) * ef;

  integralSum += ef * dt;

  if (fabs(kI) < 1e-12) {
    if (integralSum > maxIntegral) integralSum = maxIntegral;
    if (integralSum < -maxIntegral) integralSum = -maxIntegral;
  } else {
    double maxIsum = (maxIntegral / fabs(kI));
    if (integralSum > maxIsum) integralSum = maxIsum;
    if (integralSum < -maxIsum) integralSum = -maxIsum;
  }

  double dTerm = 0.0;
  if (dt > 0.0 && kD != 0.0) {
    dTerm = kD * (ef - previousError) / dt;
  }

  correction = kP * ef;

  int rawLeft = round(baseSpeed - correction);
  int rawRight = round(baseSpeed + correction);

  leftSpeed = constrain(rawLeft, minSpeed, maxSpeed);
  rightSpeed = constrain(rawRight, minSpeed, maxSpeed);

  robot.setLeftMotors(leftSpeed);
  robot.setRightMotors(rightSpeed);

  previousError = ef;
  lastMillis = now;
}

void resetPID() {
  lastMillis = millis();
  e = 0.0;
  ef = 0.0;
  integralSum = 0.0;
  previousError = 0.0;
  leftSpeed = 0;
  rightSpeed = 0;
  correction = 0.0;
}

void emergencyControl() {
  int dist = robot.getDistance();

  if (!robot.isEmergencyStopped() && dist <= dStop) {
    robot.engageEmergencyStop();
  }

  if (robot.isEmergencyStopped() && dist >= (dStop + deltaD)) {
    robot.releaseEmergencyStop();
  }
}

void Turn(int steps, int time, int xDirection, int yDirection) {
  int x[4] = { baseSpeed, baseSpeed, baseSpeed, baseSpeed };
  int y[4] = { baseSpeed, -baseSpeed, baseSpeed, -baseSpeed };
  for (int i = 0; i < 4; i++) {
    y[i] = y[i] * xDirection;
    x[i] = x[i] * yDirection;
  }
  robot.move(x);
  delay(1000);
  for (int i = 0; i < steps; i++) {
    robot.move(x);
    delay(time);
    robot.move(y);
    delay(time);
  }
  robot.move(x);
  delay(1000);
}

void threePointTurn() {
  int steps = 19, time = 70, xDirection = -1, yDirection = 1;
  int x[4] = { baseSpeed, baseSpeed, baseSpeed, baseSpeed };
  int y[4] = { baseSpeed, -baseSpeed, baseSpeed, -baseSpeed };
  for (int i = 0; i < 4; i++) {
    y[i] = y[i] * xDirection;
    x[i] = x[i] * yDirection;
  }
  robot.move(x);
  delay(1000);
  for (int i = 0; i < steps; i++) {
    robot.move(x);
    delay(time);
    robot.move(y);
    delay(time);
  }
  xDirection = -1;
  yDirection = -1;
  int x1[4] = { baseSpeed, baseSpeed, baseSpeed, baseSpeed };
  int y1[4] = { baseSpeed, -baseSpeed, baseSpeed, -baseSpeed };
  for (int i = 0; i < 4; i++) {
    y1[i] = y1[i] * xDirection;
    x1[i] = x1[i] * yDirection;
  }
  for (int i = 0; i < steps; i++) {
    robot.move(x1);
    delay(time);
    robot.move(y1);
    delay(time);
  }
  for (int i = 0; i < 4; i++)
    x1[i] = x1[i] * -1;
  robot.move(x1);
  delay(1000);
}

void park() {
  int steps = 10, time = 70, xDirection = -1, yDirection = 1;
  int x[4] = { baseSpeed, baseSpeed, baseSpeed, baseSpeed };
  int y[4] = { baseSpeed, -baseSpeed, baseSpeed, -baseSpeed };
  for (int i = 0; i < 4; i++) {
    y[i] = y[i] * xDirection;
    x[i] = x[i] * yDirection;
  }
  for (int i = 0; i < steps; i++) {
    robot.move(x);
    delay(time);
    robot.move(y);
    delay(time);
  }
  steps = 26;
  xDirection = -1;
  yDirection = 1;
  for (int i = 0; i < 4; i++) {
    y[i] = y[i] * xDirection;
    x[i] = x[i] * yDirection;
  }
  for (int i = 0; i < steps; i++) {
    robot.move(x);
    delay(time);
    robot.move(y);
    delay(time);
  }
  for (int i = 0; i < 4; i++)
    x[i] = baseSpeed;
  robot.move(x);
  delay(500);
  robot.stopAll();
}
