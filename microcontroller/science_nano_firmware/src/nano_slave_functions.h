//
// Created by danielc on 15/03/18.
//

#ifndef NANO_SLAVE_FUNCTIONS_H
#define NANO_SLAVE_FUNCTIONS_H


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

/* -- Dust Sensor -- */
int Filter(int m);          //Private function
/* -- //Dust Sensor -- */

/* -- Geiger Counter -- */
void tube_impulse();
/* -- //Geiger Counter -- */

#endif //NANO_SLAVE_FUNCTIONS_H
