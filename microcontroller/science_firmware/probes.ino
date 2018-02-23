// #include <Arduino.h>

// Preamble and setup
/* -- Barometer, Altometer, Temperature -- */
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
/* -- Barometer, Altometer, Temperature -- */

/* -- Anemometer -- */
//  Arduino Wind Speed Meter Anemometer mph – Adafruit anemometer (product ID 1733).
//  Modified code created March 2016 from original code created by Joe Burg 11th November
//  2014 at http://www.hackerscapes.com/ with help from Adafruit forum users shirad

int serial_in;
double x = 0;
double y = 0;
const int sensorPin = A1;                      //Defines the pin that the anemometer output is connected to
int sensorValue = 0;                           //Variable stores the value direct from the analog pin
float sensorVoltage = 0;                       //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float windSpeed = 0;                           // Wind speed in meters per second (m/s)
float voltageConversionConstant = 0.004882814; //This constant maps the value provided from the analog read function, which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
//int sensorDelay = 2000;                      //Delay between sensor readings, measured in milliseconds (ms)

//  Anemometer Technical Variables
//  The following variables correspond to the anemometer sold by Adafruit, but could be modified to fit other anemometers.
float voltage_Min = .4;                        // Mininum output voltage from anemometer in mV.
float windSpeedMin = 0;                        // Wind speed in meters/sec corresponding to minimum voltage
float voltageMax = 2.0;                        // Maximum output voltage from anemometer in mV.
float windSpeedMax = 32;                       // Wind speed in meters/sec corresponding to maximum voltage
/* -- Anemometer -- */

/* -- EC -- */
#include <OneWire.h>
#include <EEPROM.h>
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength + 1];                                        // store the serial command
byte receivedBufferIndex = 0;

#define ecSensorPin  A2                                                               //EC Meter analog output,pin on analog 1
#define ds18b20Pin  4                                                                 //DS18B20 signal, pin on digital 2

#define SCOUNT  100                                                                   // sum of sample point
int analogBuffer[SCOUNT];                                                             //store the analog value read from ADC
int analogBufferIndex = 0;

#define compensationFactorAddress 8                                                   //the address of the factor stored in the EEPROM
float compensationFactor;

#define VREF 5000                                                                     //for arduino uno, the ADC reference is the power(AVCC), that is 5000mV

boolean enterCalibrationFlag = 0;
float temperature, ECvalue, ECvalueRaw, ec, ec_tempC;

OneWire ds(ds18b20Pin);
/* -- EC -- */

/* -- Dissolved Oxygen -- */
//This code was written to be easy to understand.
//Modify this code as you see fit.
//This code will output data to the Arduino serial monitor.
//Type commands into the Arduino serial monitor to control the D.O. circuit.
//This code was written in the Arduino 1.8.3 IDE
//An Arduino UNO was used to test this code.
//This code was last tested 6/2017

#include <SoftwareSerial.h>                           //we have to include the SoftwareSerial library, or else we can't use it
#define rx 2                                          //define what pin rx is going to be
#define tx 3                                          //define what pin tx is going to be
SoftwareSerial myserial(rx, tx);                      //define how the soft serial port is going to work

String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_string_complete = false;                //have we received all the data from the PC
boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product
float DO;                                             //holds floating point representing the DO
/* -- Dissolved Oxygen -- */

void setup() {
  Serial.begin(115200);

  /* -- Barometer, Altometer, Temperature -- */
  //Serial.println("Adafruit_MPL3115A2 test!");
  /* -- Barometer, Altometer, Temperature -- */

  /* -- Aneometer -- */
  //delay(2500);
  /* -- Aneometer -- */

  /* -- EC -- */
  readCharacteristicValues();                         //read the compensationFactor
  /* -- EC -- */

  /* -- Dissolved Oxygen -- */
  Serial.begin(115200);                               //set baud rate for the hardware serial port_0 to *9600
  myserial.begin(9600);                               //set baud rate for the software serial port to *9600 <Test this config>
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);                           //set aside some bytes for receiving data from Atlas Scientific product
  /* -- Dissolved Oxygen -- */

}

/* --Dissolved Oxygen -- */
void serialEvent() {                                  //if the hardware serial port_0 receives a char
  inputstring = Serial.readStringUntil(13);           //read the string until we see a <CR>
  input_string_complete = true;                       //set the flag used to tell if we have received a completed string from the PC
}
/* --Dissolved Oxygen -- */

void printStuff(float pascals, float altm, float tempC, int hydrogen, float windSpeed, float ec, float ec_tempC, float DO);

void loop() {

  //barometer check
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }

  /* barometer get data */
  float pascals = baro.getPressure();
  float altm = baro.getAltitude();
  float tempC = baro.getTemperature();
  /* barometer get data */

  /* hydrogen get data */
  int hydrogen  = analogRead(0);
  /* hydrogen get data */

  /* Anemometer get data */
  sensorValue = analogRead(sensorPin);                     //Get a value between 0 and 1023 from the analog pin connected to the anemometer
  sensorVoltage = sensorValue * voltageConversionConstant; //Convert sensor value to actual voltage
  //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer
  if (sensorVoltage <= voltage_Min)
    windSpeed = 0;
  else
    windSpeed  = (sensorVoltage - voltage_Min) * windSpeedMax / (voltageMax - voltage_Min) * 2.23694;

  x = windSpeed;
  if (x >= y)
    y = x;
  else
    y = y;

  //delay(sensorDelay);
  /* Anemometer get data */

  /* EC get data */
  if (serialDataAvailable() > 0) {
    byte modeIndex = uartParse();
    ecCalibration(modeIndex);                                                         // If the correct calibration command is received, the calibration function should be called.
  }

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 30U) {                                       //every 30ms,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(ecSensorPin);                        //read the analog value and store into the buffer,every 40ms
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }

  static unsigned long tempSampleTimepoint = millis();
  if (millis() - tempSampleTimepoint > 850U) {                                        // every 1.7s, read the temperature from DS18B20
    tempSampleTimepoint = millis();
    temperature = readTemperature();                                                  // read the current temperature from the  DS18B20
    ec_tempC = temperature;                                                           // For printing
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000U) {
    printTimepoint = millis();
    float AnalogAverage = getMedianNum(analogBuffer, SCOUNT);                         // read the stable value by the median filtering algorithm
    float averageVoltage = AnalogAverage * (float)VREF / 1024.0;
    if (temperature == -1000) {
      temperature = 25.0;                                                             // when no temperature sensor ,temperature should be 25^C default
      ec_tempC = temperature;                                                         // For printing
      //Serial.print(temperature, 1);
      //Serial.print(F("^C(default)    EC:"));
    } else {
      //Serial.print(temperature, 1);                                                 //current temperature
      //Serial.print(F("^C             EC:"));
    }
    float TempCoefficient = 1.0 + 0.0185 * (temperature - 25.0);                      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge = (float)averageVoltage / TempCoefficient;
    if (CoefficientVolatge < 150)Serial.println(F("No solution!"));                   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    else if (CoefficientVolatge > 3300)Serial.println(F("Out of the range!"));        //>20ms/cm,out of the range
    else {
      if (CoefficientVolatge <= 448)ECvalue = 6.84 * CoefficientVolatge - 64.32;      //1ms/cm<EC<=3ms/cm
      else if (CoefficientVolatge <= 1457)ECvalue = 6.98 * CoefficientVolatge - 127;  //3ms/cm<EC<=10ms/cm
      else ECvalue = 5.3 * CoefficientVolatge + 2278;                                 //10ms/cm<EC<20ms/cm
      ECvalueRaw = ECvalue / 1000.0;
      ECvalue = ECvalue / compensationFactor / 1000.0;                                //after compensation,convert us/cm to ms/cm
      //Serial.print(ECvalue, 2);                                                     //two decimal
      //Serial.print(F("ms/cm"));
      ec = ECvalue;                                                                   // For printing
      if (enterCalibrationFlag) {                                                     // in calibration mode, print the voltage to user, to watch the stability of voltage
        //Serial.print(F("            Factor:"));
        //Serial.print(compensationFactor);
      }
    }
  }
  /* EC get data */

  /* DO get data */
  if (input_string_complete) {                        //if a string from the PC has been received in its entirety
    myserial.print(inputstring);                      //send that string to the Atlas Scientific product
    myserial.print('\r');                             //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
  }

  if (myserial.available() > 0) {                     //if we see that the Atlas Scientific product has sent a character
    char inchar = (char)myserial.read();              //get the char we just received
    sensorstring += inchar;                           //add the char to the var called sensorstring
    if (inchar == '\r') {                             //if the incoming character is a <CR>
      sensor_string_complete = true;                  //set the flag
    }
  }

  if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
    Serial.println(sensorstring);                     //send that string to the PC's serial monitor
    //uncomment this section to see how to convert the DO reading from a string to a float
    if (isdigit(sensorstring[0])) {                   //if the first character in the string is a digit
      DO = sensorstring.toFloat();                    //convert the string to a floating point number so it can be evaluated by the Arduino
      /*
        if (DO >= 6.0) {                                //if the DO is greater than or equal to 6.0
        Serial.println("high");                       //print "high" this is demonstrating that the Arduino is evaluating the DO as a number and not as a string
        }
        if (DO <= 5.99) {                               //if the DO is less than or equal to 5.99
        Serial.println("low");                        //print "low" this is demonstrating that the Arduino is evaluating the DO as a number and not as a string
        }
      */
    }
    sensorstring = "";                                //clear the string
    sensor_string_complete = false;                   //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
  }
  /* DO get data */

  printStuff(pascals, altm, tempC, hydrogen, windSpeed, ec, ec_tempC, DO);
  delay(250);
}

void printStuff(float pascals, float altm, float tempC, int hydrogen, float windSpeed, float ec, float ec_tempC, float DO) {
  /* barometer, altimeter, temperature */
  Serial.print("Pressure      : "); Serial.print(pascals); Serial.println(" Pa");               //output pressure (Pa)
  Serial.print("Altitude      : "); Serial.print(altm); Serial.println(" meters");              //output altitude (m)
  Serial.print("Air Temp      : "); Serial.print(tempC); Serial.println("*C");                  //output temperature from adafruit unit (C)

  /* hydrogen */
  Serial.print("hydrogen      : "); Serial.print(hydrogen, DEC); Serial.println(" ppm");        //output [hydrogen] (ppm)

  /* EC */
  Serial.print("EC Value      : "); Serial.print(ec, 2); Serial.println(" mS/cm");              //output EC (mS/cm)
  Serial.print("EC Temp       : "); Serial.print(ec_tempC); Serial.println(" *C");              //output temperature from ec temp probe (C)

  /* Anemometer */
  //Serial.print("Voltage       : "); Serial.println(sensorVoltage);                              //output anemometer voltage
  Serial.print("Wind Speed    : "); Serial.print(windSpeed); Serial.println(" mph");            //output windspeed (mph)

  /* Dissolved Oxygen */
  Serial.print("Dissolved O2  : "); Serial.print(DO); Serial.println(" mg/L");                  //output DO (mg/L)
}

/* -- EC  Fcuntions and Methods -- */
boolean serialDataAvailable(void) {
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
  char *receivedBufferPtr;
  static boolean ecCalibrationFinish = 0;
  float factorTemp;
  switch (mode) {
    case 0:
      if (enterCalibrationFlag)
        Serial.println(F("Command Error"));
      break;

    case 1:
      enterCalibrationFlag = 1;
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
        factorTemp = ECvalueRaw / 12.88;
        //if((factorTemp>0.85) && (factorTemp<1.15))
        {
          Serial.println();
          Serial.println(F(">>>Confrim Successful<<<"));
          Serial.println();
          compensationFactor = factorTemp;
          ecCalibrationFinish = 1;
        }
        //else
        //{
        //  Serial.println();
        //  Serial.println(F(">>>Confirm Failed,Try Again<<<"));
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
          Serial.print(F(">>>Calibration Failed"));
        }
        Serial.println(F(",Exit Calibration Mode<<<"));
        Serial.println();
        ecCalibrationFinish = 0;
        enterCalibrationFlag = 0;
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
  if (EEPROM.read(compensationFactorAddress) == 0xFF && EEPROM.read(compensationFactorAddress + 1) == 0xFF && EEPROM.read(compensationFactorAddress + 2) == 0xFF && EEPROM.read(compensationFactorAddress + 3) == 0xFF) {
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
    if ( !ds.search(addr)) {
      // Serial.println("no more sensors on chain, reset search!");
      ds.reset_search();
      return -1000;
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      //  Serial.println("CRC is not valid!");
      return -1000;
    }
    if ( addr[0] != 0x10 && addr[0] != 0x28) {
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
/* -- EC  Fcuntions and Methods -- */
