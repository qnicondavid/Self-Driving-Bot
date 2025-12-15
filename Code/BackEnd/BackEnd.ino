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
};

class Robot {
  private:
    IrSensor &irLeft, &irRight;
    UrSensor &ur;

    Motor &FL, &FR, &BL, &BR;

    Microcontroller &mcu;

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
};

Microcontroller mcu(8,7,4,2,"Group 19","Group 19");

IrSensor irLeft(A1, A0);
IrSensor irRight(A3, A2);

UrSensor ur(0, 1);

Motor FL(6, 5, false);   
Motor FR(11, 12, true);  
Motor BL(A4, A5, false); 
Motor BR(9, 10, true);

Robot robot(irLeft, irRight, ur, FL, FR, BL, BR, mcu);

void setup() {
  mcu.enableWiFi();   
}

void loop() {

}