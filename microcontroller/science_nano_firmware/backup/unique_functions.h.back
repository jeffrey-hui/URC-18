//
// Created by danielc on 11/03/18.
// Probe and Sensor Unique Functions


#ifndef SCIENCE_FIRMWARE_UNIQUE_FUNCTIONS_H
#define SCIENCE_FIRMWARE_UNIQUE_FUNCTIONS_H

/* -- pH Probe -- */
double averageArray(int *arr, int number);
/* -- //pH Probe -- */

/* -- EC -- */
boolean serialDataAvailable(void);
byte uartParse();

void ecCalibration(byte mode);
int getMedianNum(int bArray[], int iFilterLen);
void readCharacteristicValues();
float readTemperature();    //returns the temperature from one DS18B20 in DEG Celsius
/* -- //EC -- */

/* -- Dissolved Oxygen -- */
void serialEvent();
/* -- //Dissolved Oxygen -- */

/* -- Dust Sensor -- */
int Filter(int m);          //Private function
/* -- //Dust Sensor -- */

/* -- Geiger Counter -- */
void tube_impulse();
/* -- //Geiger Counter -- */

#endif //SCIENCE_FIRMWARE_UNIQUE_FUNCTIONS_H


