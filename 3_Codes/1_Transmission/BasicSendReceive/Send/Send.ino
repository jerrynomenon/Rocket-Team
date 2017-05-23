/*
McGill Rocket Team - 2014/2015
Author: Haris Bestowo
Started: 3 May 2015 
Updated: 14 May 2015

This is the code for the sending Arduino. 
The sending Arduino will process the voltage from the analog pin. 
Afterwards, it will send the readings serially through XBee.
*/

// set up pin:
//const int sensorPin = A0;

void setup() {
  // put your setup code here, to run once:
  // establish serial connection:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // read the input the analog pin:
  //int sensorValue = analogRead(sensorPin);
  
  //Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //float voltage = sensorValue * (5.0 / 1023.0);
  
  //send out the value you read through serial connection:
  Serial.println("This text will be sent serially to the receiving end");
}
