/*
 * AUTHOR: Daniel Cresta
 * DATE: March 15, 2018
 *
 * Arduino Science Nano
 *  - i2C slave for non-i2C Science Sensors
 *
 * Sensors Controlled:
 *  -pH Probe           -> nano : pHValue
 *  -EC, Temp Probe     -> nano : ECValue, ECTemp
 *  -Temp/Hum           -> nano : SoilTemp, SoilHumi
 *  -Dust Sensor        -> nano : DustDens
 *  -Geiger Counter     -> nano : GeigeCPM
 *  -Hydrogen           -> nano : HydroPPM
 *  -Anemometer         -> nano : WindVelo
 */

/*** Preamble and Initialization ***/

#include <Arduino.h>
#include <OneWire.h>                //For EC temperature probe
#include <EEPROM.h>                 //For EC probe
#include <SHT1x.h>                  //For Temp/Hum Sensor
#include <SPI.h>                    //For Geiger Counter
#include "nano_slave_functions.h"


/* -- //Preamble -- */

/* -- i2C Protocol -- */
#include <Wire.h>
#define NANO_ADDRESS 0x10

int call = 0;

///WILL THIS WORK?
/*
short _pHValue;
short _ECValue;
short _ECTemp;
short _SoilTemp;
short _SoilHumi;
short _DustDens;
short _GeigeCPM;
float _HydroPPM;
short _WindVelo;

/* -- //i2C Protocol -- */

/* -- pH Probe -- */
#define pHPin A3                //pH meter Analog output to Arduino Analog Input 3
#define pHLED 13                ///REMOVE if probe works without LED

#define pHOffset 0.00           ///ADJUST deviation compensate
#define pHSamplingInterval 20   //Time between measurements
#define pHArrayLength  40       //times of collection

int pHArray[pHArrayLength];     //Store the average value of the sensor feedback
int pHArrayIndex = 0;           //Start storing at index 0
/* -- //pH Probe -- */

/* -- EC, Temp -- */
#define ecSensorPin  A2                          //EC Meter analog output,pin on analog 1
#define ecTempPin  5                             //DS18B20 signal, pin on digital 2

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write((address)+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read((address)+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength + 1];   //store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT 100                               //sum of sample point
int analogBuffer[SCOUNT];                        //store the analog value read from ADC
int analogBufferIndex = 0;

#define compensationFactorAddress 8              //the address of the factor stored in the EEPROM
float compensationFactor;

#define VREF 5000                                //for arduino uno, the ADC reference is the power(AVCC), that is 5000mV

boolean enterCalibrationFlag = false;
float ECValue, ECValueRaw, ECTemp;

OneWire ds(ecTempPin);
/* -- //EC, Temp -- */

/* -- Temperature and Humidity Probe -- */
#define THdataPin 3
#define THclockPin 4
SHT1x sht1x(THdataPin, THclockPin);

float SoilTemp;
float SoilHumi;
/* -- //Temperature and Humidity Probe -- */

/* -- Dust Sensor -- */
#define COV_RATIO 0.2           //ug/mmm / mv
#define NO_DUST_VOLTAGE 400     //mv
#define SYS_VOLTAGE 5000

// I/O define
#define dustLED 6               //Drive the led of sensor
#define dustOut A6              //Analog input

// variables
float DustDens, voltage;
int dustADCValue;
/* -- //Dust Sensor -- */

/* -- Geiger Counter */
#define GeigPin 3               //Geiger Counter Pin (with interrupt)
#define LOG_PERIOD 20000        //Logging period in milliseconds, recommended value 15000-60000.
#define MAX_PERIOD 60000        //Maximum logging period without modifying this sketch

unsigned long counts;           //variable for GM Tube events
unsigned long GeigeCPM;         //variable for CPM
unsigned int multiplier;        //variable for calculation CPM in this sketch
unsigned long previousMillis;   //variable for time measurement
/* -- //Geiger Counter */

/* -- Hydrogen -- */
#define H2Pin A0                //Hydrogen Sensor Pin
int hydroPPM = 0;
/* -- //Hydrogen -- */

/* -- Anemometer -- */
#define anemPin A1                              //Defines the pin that the anemometer output is connected to

int anemValue = 0;                              //Variable stores the value direct from the analog pin
float anemVoltage = 0;                          //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float WindVelo = 0;                             //Wind speed in meters per second (m/s)
float voltageConversionConstant = 0.004882814;  //This constant maps the value provided from the analog read function, which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
float voltage_Min = 0.48;                       //Mininum output voltage from anemometer in mV.
float windSpeedMin = 0;                         //Wind speed in meters/sec corresponding to minimum voltage
float voltageMax = 2.0;                         //Maximum output voltage from anemometer in mV.
float windSpeedMax = 32;                        //Wind speed in meters/sec corresponding to maximum voltage

#define ADD_BASE 0x40
#define ADD_PIN_1 7
#define ADD_PIN_2 8
int ADDRESS;
float pHValue = 0;
void set_address(){
    pinMode(ADD_PIN_1, INPUT);
    pinMode(ADD_PIN_2, INPUT);
    ADDRESS = ADD_BASE;
    ADDRESS += digitalRead(ADD_PIN_1);
    ADDRESS += digitalRead(ADD_PIN_2) * 2;
}

/* -- //Anemometer -- */

#define setFloatIn(data_val, ind, val) memcpy(data_val + (ind * sizeof(float)), &val, sizeof(float))


void receiveCall(int bytes) {
    int call_p = Wire.read();
    const int NUM_DATAS = 8;
    if (call_p == 0x01) {
        char data_values[sizeof(float) * NUM_DATAS];
        setFloatIn(data_values, 0, pHValue);
        setFloatIn(data_values, 1, ECTemp);
        setFloatIn(data_values, 2, ECValue);
        setFloatIn(data_values, 3, SoilHumi);
        setFloatIn(data_values, 4, SoilTemp);
        setFloatIn(data_values, 5, DustDens);
        setFloatIn(data_values, 6, GeigeCPM);
        setFloatIn(data_values, 7, WindVelo);
        Wire.write(data_values, sizeof(float) * NUM_DATAS);
    }
}

void setup(void){
    set_address();
    /* -- I2C Protocol -- */
    Wire.begin(ADDRESS);
    Wire.onReceive(receiveCall);
    /* -- I2C Protocol -- */

    /* -- pH Probe -- */
    pinMode(pHLED, OUTPUT);                     ///REMOVE if probe works without LED
    /* -- pH Probe -- */

    /* -- EC, Temp -- */
    readCharacteristicValues();            //read the compensationFactor
    /* -- //EC, Temp -- */

    /* -- Dust Sensor -- */
    pinMode(dustLED, OUTPUT);                   ///REMOVE if works without LED
    digitalWrite(dustLED, LOW);                 ///REMOVE if works without LED; dustLED default closed
    /* -- //Dust Sensor -- */

    /* -- Geiger Counter -- */
    counts = 0;
    GeigeCPM = 0;
    multiplier = MAX_PERIOD / LOG_PERIOD;       //calculating multiplier, depend on your log period
    attachInterrupt(digitalPinToInterrupt(GeigPin), tube_impulse, FALLING);      //define external interrupts
    /* -- //Geiger Counter -- */

    Serial.begin(9600);
}

void loop(void) {

    /* -- pH Probe -- */
    static unsigned long samplingTime = millis();
    static float pHVoltage;
    if (millis() - samplingTime > pHSamplingInterval) {
        pHArray[pHArrayIndex] = analogRead(pHPin);
        if (pHArrayIndex == pHArrayLength)pHArrayIndex = 0;
        pHVoltage = averageArray(pHArray, pHArrayLength) * 5.0 / 1024;
        pHValue = 3.5 * pHVoltage + pHOffset;
        samplingTime = millis();
        pHArrayIndex++;
    }
    /* -- //pH Probe -- */

    /* -- EC, Temp -- */

    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 30U) {                                           //every 30ms,read the analog value from the ADC
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(ecSensorPin);                          //read the analog value and store into the buffer,every 40ms
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
            analogBufferIndex = 0;
    }

    static unsigned long tempSampleTimepoint = millis();
    if (millis() - tempSampleTimepoint > 850U) {                                            // every 1.7s, read the temperature from DS18B20
        tempSampleTimepoint = millis();
        ECTemp = readTemperature();                                                         // read the current temperature from the  DS18B20
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 1000U) {
        printTimepoint = millis();
        float AnalogAverage = getMedianNum(analogBuffer, SCOUNT);                           // read the stable value by the median filtering algorithm
        float averageVoltage = AnalogAverage * (float) VREF / 1024.0;
        if (ECTemp == -1000) {
            ECTemp = 25.0;                                                                  // when no temperature sensor, temperature should be 25^C default
        }
        float TempCoefficient = 1.0 + 0.0185 * (ECTemp - 25.0);                             //temperature compensation formula: FinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
        float CoefficientVolatge = (float) averageVoltage / TempCoefficient;
        if (CoefficientVolatge < 150) {}
        else if (CoefficientVolatge > 3300) {}          //>20ms/cm,out of the range
        else {
            if (CoefficientVolatge <= 448)ECValue = 6.84 * CoefficientVolatge - 64.32;      //1ms/cm<EC<=3ms/cm
            else if (CoefficientVolatge <= 1457)ECValue = 6.98 * CoefficientVolatge - 127;  //3ms/cm<EC<=10ms/cm
            else ECValue = 5.3 * CoefficientVolatge + 2278;                                 //10ms/cm<EC<20ms/cm
            ECValueRaw = ECValue / 1000.0;
            ECValue = ECValue / compensationFactor / 1000.0;                                //after compensation,convert us/cm to ms/cm
        }
    }
    /* -- //EC, Temp -- */

    /* -- Temperature and Humidity Probe -- */
    SoilTemp = sht1x.readTemperatureC();
    SoilHumi = sht1x.readHumidity();
    /* -- Temperature and Humidity Probe -- */

    /* -- Dust Sensor -- */
    dustADCValue = analogRead(dustOut);
    dustADCValue = Filter(dustADCValue);

    //covert voltage (mv)
    voltage = (SYS_VOLTAGE / 1024.0) * dustADCValue * 11;

    //voltage to DustDens
    if (voltage >= NO_DUST_VOLTAGE) {
        voltage -= NO_DUST_VOLTAGE;
        DustDens = voltage * COV_RATIO;
    } else
        DustDens = 0;
    /* -- //Dust Sensor -- */

    /* -- Geiger Counter -- */
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= LOG_PERIOD) {
        previousMillis = currentMillis;
        GeigeCPM = counts * multiplier;
        counts = 0;
    }
    /* -- //Geiger Counter -- */

    /* -- Hydrogen -- */
    //HydroPPM = analogRead(H2Pin);
    /* -- //Hydrogen -- */

    /* -- Anemometer -- */
    anemValue = analogRead(anemPin);                     //Get a value between 0 and 1023 from the analog pin connected to the anemometer
    anemVoltage = anemValue * voltageConversionConstant; //Convert sensor value to actual voltage
    //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer
    if (anemVoltage <= voltage_Min)
        WindVelo = 0;
    else
        WindVelo = (anemVoltage - voltage_Min) * windSpeedMax / (voltageMax - voltage_Min) * 2.23694 + windSpeedMin;

    /* -- //Anemometer -- */

    /* -- I2C Protocol -- */
    /* -- //I2C Protocol -- */

    delay(2000);
}

/* -- I2C Protocol -- */

/* -- I2C Protocol -- */

//Probe and Sensor Unique Functions
/* -- pH Probe -- */
double averageArray(int *arr, int number) {
    int i;
    int max, min;
    double avg;
    long amount = 0;
    if (number <= 0) {
        return 0;
    }
    if (number < 5) {                   //less than 5, calculated directly statistics
        for (i = 0; i < number; i++) {
            amount += arr[i];
        }
        avg = amount / number;
        return avg;
    } else {
        if (arr[0] < arr[1]) {
            min = arr[0];
            max = arr[1];
        } else {
            min = arr[1];
            max = arr[0];
        }
        for (i = 2; i < number; i++) {
            if (arr[i] < min) {
                amount += min;          //arr<min
                min = arr[i];
            } else {
                if (arr[i] > max) {
                    amount += max;      //arr>max
                    max = arr[i];
                } else {
                    amount += arr[i];   //min<=arr<=max
                }
            }//if
        }//for
        avg = (double) amount / (number - 2);
    }//if
    return avg;
}
/* -- //pH Probe -- */

/* -- EC, Temp -- */
boolean serialDataAvailable() {
    char receivedChar;
    static unsigned long receivedTimeOut = millis();
    while (Serial.available() > 0) {
        if (millis() - receivedTimeOut > 500U) {
            receivedBufferIndex = 0;
            memset(receivedBuffer, 0, (ReceivedBufferLength + 1));
        }
        receivedTimeOut = millis();
        receivedChar = Serial.read();
        if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength) {
            receivedBufferIndex = 0;
            strupr(receivedBuffer);
            return true;
        } else {
            receivedBuffer[receivedBufferIndex] = receivedChar;
            receivedBufferIndex++;
        }
    }
    return false;
}

byte uartParse() {
    byte modeIndex = 0;
    if (strstr(receivedBuffer, "CALIBRATION") != NULL)
        modeIndex = 1;
    else if (strstr(receivedBuffer, "EXIT") != NULL)
        modeIndex = 3;
    else if (strstr(receivedBuffer, "CONFIRM") != NULL)
        modeIndex = 2;
    return modeIndex;
}

void ecCalibration(byte mode) {
    static boolean ecCalibrationFinish = false;
    float factorTemp;
    switch (mode) {
        case 0:
            if (enterCalibrationFlag)
                Serial.println(F("Command Error"));
            break;

        case 1:
            enterCalibrationFlag = true;
            ecCalibrationFinish = 0;
            Serial.println();
            Serial.println(F(">>>Enter Calibration Mode<<<"));
            Serial.println(F(">>>Please put the probe into the 12.88ms/cm buffer solution<<<"));
            Serial.println(F(">>>When Reading Stabilizes, 'confirm'<<<"));
            Serial.println();
            break;

            //Lines commented out due to restrictions on measurements: For single point calibration
        case 2:
            if (enterCalibrationFlag) {
                //NOTE: Modified for single point calibration.
                factorTemp = ECValueRaw / 12.88;
                //if((factorTemp>0.85) && (factorTemp<1.15))
                {
                    Serial.println();
                    Serial.println(F(">>>Confirm Successful<<<"));
                    Serial.println();
                    compensationFactor = factorTemp;
                    ecCalibrationFinish = 1;
                }
                //else
                //{
                //  Serial.println();
                //  Serial.println(F(">>>Confirm calibration,Try Again<<<"));
                //  Serial.println();
                //  ecCalibrationFinish = 0;
                //}
            }
            break;

        case 3:
            if (enterCalibrationFlag) {
                Serial.println();
                if (ecCalibrationFinish) {
                    EEPROM_write(compensationFactorAddress, compensationFactor);
                    Serial.print(F(">>>Calibration Successful"));
                } else {
                    Serial.print(F(">>>Calibration FadustLED"));
                }
                Serial.println(F(",Exit Calibration Mode<<<"));
                Serial.println();
                ecCalibrationFinish = false;
                enterCalibrationFlag = false;
            }
            break;
    }
}

int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++) {
        bTab[i] = bArray[i];
    }
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}

void readCharacteristicValues() {
    EEPROM_read(compensationFactorAddress, compensationFactor);
    if (EEPROM.read(compensationFactorAddress) == 0xFF && EEPROM.read(compensationFactorAddress + 1) == 0xFF &&
        EEPROM.read(compensationFactorAddress + 2) == 0xFF && EEPROM.read(compensationFactorAddress + 3) == 0xFF) {
        compensationFactor = 1.0;                               // If the EEPROM is new, the compensationFactorAddress is 1.0(default).
        EEPROM_write(compensationFactorAddress, compensationFactor);
    }
}

//returns the temperature from one DS18B20 in DEG Celsius
float readTemperature() {
    static byte data[12], addr[8];
    static float TemperatureSum = 25;
    static boolean ch = 0;
    if (!ch) {
        if (!ds.search(addr)) {
            // Serial.println("no more sensors on chain, reset search!");
            ds.reset_search();
            return -1000;
        }
        if (OneWire::crc8(addr, 7) != addr[7]) {
            //  Serial.println("CRC is not valid!");
            return -1000;
        }
        if (addr[0] != 0x10 && addr[0] != 0x28) {
            //  Serial.print("Device is not recognized!");
            return -1000;
        }
        ds.reset();
        ds.select(addr);
        ds.write(0x44, 1); // start conversion, with parasite power on at the end
    } else {
        byte present = ds.reset();
        ds.select(addr);
        ds.write(0xBE); // Read Scratchpad
        for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds.read();
        }
        ds.reset_search();
        byte MSB = data[1];
        byte LSB = data[0];
        float tempRead = ((MSB << 8) | LSB); //using two's compliment
        TemperatureSum = tempRead / 16;
    }
    ch = !ch;
    return TemperatureSum;
}
/* -- //EC, Temp -- */

/* -- Dust Sensor -- */
//Private function
int Filter(int m) {
    static int flag_first = 0, _buff[10], sum;
    const int _buff_max = 10;
    int i;

    if (flag_first == 0) {
        flag_first = 1;

        for (i = 0, sum = 0; i < _buff_max; i++) {
            _buff[i] = m;
            sum += _buff[i];
        }
        return m;
    } else {
        sum -= _buff[0];
        for (i = 0; i < (_buff_max - 1); i++) {
            _buff[i] = _buff[i + 1];
        }
        _buff[9] = m;
        sum += _buff[9];

        i = sum / 10.0;
        return i;
    }
}
/* -- //Dust Sensor -- */

/* -- Geiger Counter -- */
void tube_impulse() {                                  //subprocedure for capturing events from Geiger Kit
    counts++;
}
/* -- //Geiger Counter -- */

