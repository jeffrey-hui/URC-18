
#include <SoftwareSerial.h>             //we have to include the SoftwareSerial library, or else we can't use it
#include <Servo.h>                      //TODO ADD DEPENDENCIES
#define rx 2                            //define what pin rx is going to be
#define tx 3                            //define what pin tx is going to be
#define ADD_BASE 0x40
#define ADD_PIN_1 7
#define ADD_PIN_2 8
#define SERVO 9
//define how the soft serial port is going to work
//a string to hold incoming data from the PC
//a string to hold the data from the Atlas Scientific product
//have we received all the data from the PC
//have we received all the data from the Atlas Scientific product
SoftwareSerial myserial(rx, tx);        
String inputstring = "";                
String sensorstring = "";               
boolean input_string_complete = false;  
boolean sensor_string_complete = false; 
float DO;                               

//if the hardware serial port_0 receives a char
//read the string until we see a <CR>
//set the flag used to tell if we have received a completed string from the PC
void serialEvent()
{                                           
  inputstring = Serial.readStringUntil(13); 
  input_string_complete = true;             
}
void set_address(){
    pinMode(ADD_PIN_1, INPUT);
    pinMode(ADD_PIN_2, INPUT);
    ADDRESS = ADD_BASE;
    ADDRESS += digitalRead(ADD_PIN_1);
    ADDRESS += digitalRead(ADD_PIN_2) * 2;
}
void writeServo(Servo s){
  while(1 < Wire.available){
    s.write(Wire.read());
    delay(1000);
  }
}

void setup()
{                          
  Servo SERVO;
  SERVO.attach(9);
  Wire.begin(ADDRESS);
  Wire.onRecieve(writeServo(SERVO));
  pinMode(SERVO,output);        
  Serial.begin(9600);       
  myserial.begin(9600);     
  inputstring.reserve(10);  
  sensorstring.reserve(30); 
}


void loop()
{ 

  if (input_string_complete)
  {                                
    myserial.print(inputstring);   
    myserial.print('\r');          
    inputstring = "";              
    input_string_complete = false; 
  }
  if (myserial.available() > 0)
  {                                      //if we see that the Atlas Scientific product has sent a character
    char inchar = (char)myserial.read(); //get the char we just received
    sensorstring += inchar;              //add the char to the var called sensorstring
    if (inchar == '\r')
    {                                //if the incoming character is a <CR>
      sensor_string_complete = true; //set the flag
    }
  }
  if (sensor_string_complete == true)
  {                               //if a string from the Atlas Scientific product has been received in its entirety
    Serial.println(sensorstring); //send that string to the PC's serial monitor
    /*                                                //uncomment this section to see how to convert the DO reading from a string to a float 
    if (isdigit(sensorstring[0])) {                   //if the first character in the string is a digit
      DO = sensorstring.toFloat();                    //convert the string to a floating point number so it can be evaluated by the Arduino
      if (DO >= 6.0) {                                //if the DO is greater than or equal to 6.0
        Serial.println("high");                       //print "high" this is demonstrating that the Arduino is evaluating the DO as a number and not as a string
      }
      if (DO <= 5.99) {                               //if the DO is less than or equal to 5.99
        Serial.println("low");                        //print "low" this is demonstrating that the Arduino is evaluating the DO as a number and not as a string
      }
    }
    */
    sensorstring = "";              //clear the string
    sensor_string_complete = false; //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
  }
}