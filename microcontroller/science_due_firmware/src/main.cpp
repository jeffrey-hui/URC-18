/*
 * AUTHOR: Daniel Cresta
 * DATE: March 15, 2018
 *
 * Arduino Science DUE
 *  - i2C slave for tx2
 *  - i2C master for non-i2C Science Sensors
 *
 * Sensors Controlled:
 *  - Barometer, Altimeter, Atm. Temperature    -> i2C
 *  - nano slave:                               -> i2C
 *      - (1) pH Probe           -> nano
 *      - (2,3) EC, Temp Probe   -> nano
 *      - (4,5) Temp/Hum         -> NOTE: The SHT1x will not interfere with i2C protocol, but will need to be run separately
 *      - (6) Dust Sensor        -> nano
 *      - (7) Geiger Counter     -> nano
 *      - (8) Hydrogen           -> nano
 *      - (9) Anemometer         -> nano
 */

#include <Arduino.h>
#include <Wire.h>

/* -- Master -- */

//Barometer, Altimeter, Atm. Temperature
#include <Adafruit_MPL3115A2.h>
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();


#define NANO_ADDRESS 0x10
#define BARO_ADDRESS 0x11

#define NANO_pH 1
#define NANO_EC 2
#define NANO_ECTemp 3
#define NANO_SoilTemp 4
#define NANO_SoilHumi 5
#define NANO_Dust 6
#define NANO_Geig 7
#define NANO_Hydr 8
#define NANO_Anem 9

/* -- //Master -- */

/* -- Slave -- */

#define TX_ADDRESS_PIN_1 45
#define TX_ADDRESS_PIN_2 43
#define TX_ADDRESS_PIN_3 44
#define TX_ADDRESS_PIN_4 42

int getAddress() {
    pinMode(TX_ADDRESS_PIN_1, INPUT);
    pinMode(TX_ADDRESS_PIN_2, INPUT);
    pinMode(TX_ADDRESS_PIN_3, INPUT);
    pinMode(TX_ADDRESS_PIN_4, INPUT);

    int address = 0x30;
    address += digitalRead(TX_ADDRESS_PIN_1);
    address += digitalRead(TX_ADDRESS_PIN_2) * 2;
    address += digitalRead(TX_ADDRESS_PIN_3) * 4;
    address += digitalRead(TX_ADDRESS_PIN_4) * 8;

    return address;
}

/* -- //Slave -- */

void setup(){

    /* -- Master -- */
    Wire1.begin();
    /* -- //Master -- */

    /* -- Slave -- */
    Wire.begin(getAddress());       //Start I2C Bus Master

    /* -- //Slave -- */
}

void loop(){

    /* -- Master --  */

    //Get data from nano
    Wire1.beginTransmission(NANO_ADDRESS);
    Wire1.write(NANO_pH);
    Wire1.read();
    Wire1.write(NANO_EC);
    Wire1.read();
    Wire1.write(NANO_ECTemp);
    Wire1.read();
    Wire1.write(NANO_SoilTemp);
    Wire1.read();
    Wire1.write(NANO_SoilHumi);
    Wire1.read();
    Wire1.write(NANO_Dust);
    Wire1.read();
    Wire1.write(NANO_Geig);
    Wire1.read();
    Wire1.write(NANO_Hydr);
    Wire1.read();
    Wire1.write(NANO_Anem);
    Wire1.read();
    Wire1.endTransmission(NANO_ADDRESS);

    //Get data from Barometer, Altimeter, Temperature
    Wire1.beginTransmission(BARO_ADDRESS);
    float pascals = baro.getPressure();
    float altm = baro.getAltitude();
    float baro_tempC = baro.getTemperature();
    Wire1.endTransmission(BARO_ADDRESS);

    /* -- //Master -- */

    /* -- Slave -- */

    /* -- //Slave -- */


}