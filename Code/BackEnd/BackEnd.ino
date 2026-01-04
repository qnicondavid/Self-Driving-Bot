#include <SPI.h>
#include <WiFi101.h>

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
    {
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

int baseSpeed  = 80;
int minSpeed   = -120;  
int maxSpeed   = 120;
int leftSpeed  = 0;
int rightSpeed = 0;

double alpha     = 0.3;   
double kP        = 100;   
double kI        = 0.05;  
double kD        = 0;   
double correction= 0.0;
double maxIntegral = 200.0;

int dStop   = 1;
int deltaD  = 1;   

class Robot {
  private:
    IrSensor &irLeft, &irRight;
    UrSensor &ur;

    Motor &FL, &FR, &BL, &BR;

    Microcontroller &mcu;

    bool inPID = false; 

    bool emergencyStop = false;

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

    void stopAll() {
      FL.stop();
      FR.stop();
      BL.stop();
      BR.stop();
    }

  void printInformation(Print &out) {
    out.println("Sensor readings:");
    out.println();

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
    out.print("EMERGENCY STOP: ");
    out.println(emergencyStop ? "ACTIVE" : "CLEAR");

    out.println();
    out.println("Motor speeds (signed):");

    out.print("Front Left: ");
    out.println(FL.getSignedSpeed());

    out.print("Front Right: ");
    out.println(FR.getSignedSpeed());

    out.print("Back Left: ");
    out.println(BL.getSignedSpeed());

    out.print("Back Right: ");
    out.println(BR.getSignedSpeed());
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
    stopPID();
  }

  bool isEmergencyStopped() {
    return emergencyStop;
  }

    int getLeftIR()  { return irLeft.readAnalog(); }
    int getRightIR() { return irRight.readAnalog(); }

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

Microcontroller mcu(8,7,4,2,"Group 19","ShutDown1");

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

    if (streamClient && streamClient.connected() && millis() - lastSend >= 500) {
        robot.printInformation(streamClient);
        streamClient.println("----------------------");
        streamClient.flush();
        lastSend = millis();
    }

    if (controlClient && controlClient.connected() && controlClient.available()) {
        requestHandler.handle(controlClient);
        delay(5);
        controlClient.stop();
    }
}

void setup() {
  mcu.enableWiFi();
}

void loop() {
  emergencyControl();
  printHandler();
  pidControl();
}

void pidControl() {
  if (robot.isEmergencyStopped()) return;
  if (!robot.isInPID()) return;

  unsigned long now = millis();
  double dt = (now - lastMillis) / 1000.0;
  if (dt <= 0.0) dt = 0.001;

  int leftSensor  = constrain(robot.getLeftIR(),  0, 1023);
  int rightSensor = constrain(robot.getRightIR(), 0, 1023);

  e  = double(leftSensor) - double(rightSensor);
  ef = alpha * e + (1.0 - alpha) * ef;

  integralSum += ef * dt;

  double maxIsum = maxIntegral / max(fabs(kI), 1e-9);
  integralSum = constrain(integralSum, -maxIsum, maxIsum);

  correction = kP * ef + kI * integralSum + kD * ((ef - previousError) / dt);

  leftSpeed  = constrain(round(baseSpeed - correction), minSpeed, maxSpeed);
  rightSpeed = constrain(round(baseSpeed + correction), minSpeed, maxSpeed);

  robot.setLeftMotors(leftSpeed);
  robot.setRightMotors(rightSpeed);

  previousError = ef;
  lastMillis = now;
}

void dBangBangControl() {
  
}

void resetPID() {
  lastMillis = millis();
  e = 0.0;      
  ef = 0.0;      
  integralSum = 0.0; 
  previousError = 0.0;
  leftSpeed  = 0;
  rightSpeed = 0;
  correction= 0.0;
}

void emergencyControl() {
  int dist = ur.getDistanceCM();

  if (!robot.isEmergencyStopped() && dist <= dStop) {
    robot.engageEmergencyStop();
  }

  if (robot.isEmergencyStopped() && dist >= (dStop + deltaD)) {
    robot.releaseEmergencyStop();
  }
}


