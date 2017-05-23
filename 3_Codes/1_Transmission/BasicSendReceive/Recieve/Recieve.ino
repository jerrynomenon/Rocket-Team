/*
McGill Rocket Team - 2014/2015
Author: Haris Bestowo
Started: 3 May 2015 
Updated: 14 May 2015

This is the code for the receiving Arduino. 
The receiving Arduino will receive the voltage data from the sending end. 
If the voltage is below a certain threshold then an LED light will turn OFF. 
Consequently, if the voltage is above that same threshold then an LED light will turn ON.

*/
// set pin numbers
const int connectionLED = 7;

void setup() {
  // put your setup code here, to run once:
  // establish serial connection:
  Serial.begin(9600);
  
  // setup pin
  pinMode(connectionLED, OUTPUT);
  // signals that connection is successful:
  digitalWrite(connectionLED, HIGH);
  delay(3000);
  digitalWrite(connectionLED, LOW);
  
  // This is the heading for Temp, Press, Altimeter sensors
  Serial.print("T(C)");
  Serial.print("\t");
  Serial.print("P(Pa)");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("P(hPa)= ");
  Serial.print("\t");
  Serial.print("Alt(m) = ");
  Serial.print("\t"); 
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()){
    
    //To get numbers from the serial transmission
    //Code doesn't work
    
    /*String data = Serial.readStringUntil('*');
    char datainChar[100];
    data.toCharArray(datainChar, 100);
    int voltage = Serial.read();
    float alt;
    float p;
    float t;
    sscanf(datainChar, "*:%f*:%f*:%f", &alt, &p, &t);
    sscanf(datainChar, "Altitude(m):%f", &alt);
    Serial.write(datainChar);*/
    
    Serial.write(Serial.read());
    
    //this section is for data analysis test
    
    /*if (alt > 43){
      digitalWrite(7, HIGH);
      delay(10);
      digitalWrite(7, LOW);
    }else if(alt < 43){
      digitalWrite(7, LOW);
      //delay(100);
    }*/
  }
}
