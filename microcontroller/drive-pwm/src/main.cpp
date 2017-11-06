//
// Created by matthew on 24/08/17.
//

#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>

Servo servos[10];
uint8_t servoCount = 0;
uint8_t lastName = 0;

const int ADDRESS = 0x30;

uint8_t addServo(uint8_t pin) {
    servos[servoCount].attach((int)pin);
    servoCount += 1;
    return servoCount - 1;
}

void resetServos() {
    if (servoCount == 0) return;
    for (int i = 0; i < servoCount; i++) {
        servos[i].detach();
    }
    //free(servos);
    servoCount = 0;
}

void setServo(uint8_t name, uint16_t micros) {
    if (name < servoCount) {
        servos[name].writeMicroseconds(micros);
    }
}

void onRecieve(int bytes) {
    //Serial.println("I getting data");
    int func = Wire.read();
    //Serial.println(func);
    if (bytes == 2 && func == 0x01) {
       // Serial.println("Adding pin");
        lastName = addServo(static_cast<uint8_t>(Wire.read()));
      //  Serial.println(lastName);
    }
    else if (bytes == 4 && func == 0x02) {
        Serial.println("Setting pin");
        uint8_t servo = static_cast<uint8_t>(Wire.read());
        uint16_t value;
        Wire.readBytes((char *)&value, 2);
      //  Serial.print("serv "); Serial.print(servo); Serial.print(" val "); Serial.println(value);
        setServo(servo, value);
    }
    else if (bytes == 1 && func == 0x03) {
        //Serial.println("Reset!");
        resetServos();
    }
    else {
        bytes--;
        while (bytes > 0) {
            Wire.read(); // scoop it up
            bytes--;
        }
    }
}

void onRequest() {
    //Serial.println("i need to send data");
    Wire.write(lastName);
}

void setup() {
  //  Serial.begin(9600); //debugging
    Serial.println("OK");
    Wire.begin(ADDRESS);
    Wire.onRequest(onRequest);
    Wire.onReceive(onRecieve);
   // Serial.println("I am running");
}

void loop() {
    //delay(200); // yaaaay
    //Serial.println("I am loop");
}
