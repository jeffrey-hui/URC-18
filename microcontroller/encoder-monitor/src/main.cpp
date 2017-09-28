#include <Encoder.h>
#include <Wire.h>

const int pinA = 1;
const int pinB = 2;
const int address = 0x31

Encoder enc(pinA, pinB);

void onRequest(){
  Wire.write(enc.read());
}
void onRecieve(int bytes){
  enc.write(Wire.read());
}
void setup(){
  Wire.begin(address);
  Wire.onRequest(onRequest);
  Wire.onRecieve(onRecieve);
}
void loop(){

}
