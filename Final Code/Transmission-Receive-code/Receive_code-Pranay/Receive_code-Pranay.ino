//McGill Rocket team - Recovery and Tracking
//Rocket transmission receiving code
//Printing the code as char

char letter;

void setup() {

  Serial.begin(9600);

}

void loop() {

  if (Serial.available()) {

    letter = Serial.read();
    Serial.print(letter);
  }

}
