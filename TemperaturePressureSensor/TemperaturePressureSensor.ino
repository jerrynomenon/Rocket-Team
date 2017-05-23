//the commented code can be used to debug via either the serial monitor or LEDs

#include <SparkFunMPL3115A2.h>          //the provided library for this sensor *note that the one used is modified (has one function added, isRead()) from the original*

MPL3115A2 barometer;

double tempAverage = 0;                 //stores the running average temperature over the last few measurements
const byte numTempMeasurements = 5;     //the number of measurements to store
double temp[numTempMeasurements];       //carries the previous measurements
unsigned int tempMeasurement = 0;       //keeps track of the number of measurements made since the last reset (used to overwrite the correct measurements when new ones are added)
byte tempMisses = 0;                    //keeps track of how many consective "misreads" there have been
double tempTolerance = 10;               //the amount a read can differ from the mean by before being registered as a misread PER SECOND (this value is scaled by measurementPeriod to get the true tolerance)
byte tempMaxMisses = 2;                 //the amount of consective misreads tolerated before deciding they are not misreads and resets the running average

//pressure equivilents of the above variables
double pressureAverage = 0;
const byte numPressureMeasurements = 5;
double pressure[numPressureMeasurements];
unsigned int pressureMeasurement = 0;
byte pressureMisses = 0;
double pressureTolerance = 15000;
byte pressureMaxMisses = 2;

unsigned long startTime;                //the time at which the program finished setting up/resetting
unsigned int timer = 1;                 //the number of measurement periods passed since the last reset
const int measurementPeriod = 1000;     //number of milliseconds per measurement

void setup() 
{
  //pinMode(12, OUTPUT);
  //pinMode(9, OUTPUT);
  //pinMode(10, OUTPUT);
  //pinMode(11, OUTPUT);
  Wire.begin();
  //Serial.begin(9600);
  barometer.begin();

  //can be used to signal if barometer is ready
  /*if(barometer.isReady())
  {
      digitalWrite(12, HIGH);
      delay(50);
      digitalWrite(12, LOW);
  }
  else
    Serial.println("Sensor not detected.");*/
  
  barometer.setModeBarometer();
  barometer.setOversampleRate(7);     //makes 2^argument measurements per read to reduce noise, can be lowered if need faster measurements (7 is the max)
  barometer.enableEventFlags();

  //gets the first measurements for temperature and calculates average
  for(int i = 0; i < numTempMeasurements; i++)
  {
    temp[i] = barometer.readTemp();
    tempAverage += temp[i];
    delay(100);
  }
  tempAverage/=numTempMeasurements;

  //same for pressure
  for(int i = 0; i < numPressureMeasurements; i++)
  {
    pressure[i] = barometer.readPressure();
    pressureAverage += pressure[i];
    delay(100);
  }
  pressureAverage/=numPressureMeasurements;
  
  startTime = millis();       //marks the starting time
}

//retakes a series of measurements to reset the temperature readings (same manner as in setup)
//can compare the previous average to the new average to signal in this function if there was a significant rise/drop in temperature
void reInitTemp()
{
  //digitalWrite(11, HIGH);
  //Serial.println("Reinitializing Temperature...");
  
  float newTemp = 0;
  for(int i = 0; i < numTempMeasurements; i++)
  {
    temp[i] = barometer.readTemp();
    newTemp += temp[i]; 
    delay(100);
  }
  tempMeasurement = 0;
  timer = 0;
  startTime = millis();
  newTemp/=numTempMeasurements;
  tempAverage = newTemp;
  tempMisses = 0;
  
  //Serial.println("Done");
  //digitalWrite(11, LOW);
}

//same as reInitTemp but for pressure
void reInitPressure()
{
  //digitalWrite(10, HIGH);
  //Serial.println("Reinitilizing Pressure...");
  
  float newPressure = 0;
  for(int i = 0; i < numPressureMeasurements; i++)
  {
    pressure[i] = barometer.readPressure();
    newPressure += pressure[i]; 
    delay(100);
  }
  pressureMeasurement = 0;
  timer = 0;
  startTime = millis();
  newPressure/=numPressureMeasurements;
  pressureAverage = newPressure;
  pressureMisses = 0;
  
  //Serial.println("Done\n");
  //digitalWrite(10, LOW);
}

void loop() 
{
  if(millis() - startTime >= (unsigned long)(timer) * measurementPeriod) //if we are due for a measurement
  {
    //digitalWrite(9, HIGH);
    //delay(50);
    //digitalWrite(9, LOW);
    
    //takes reading
    double T = barometer.readTemp();
    /*Serial.println("Temperature:");
    Serial.print("Old Average: ");
    Serial.println(tempAverage);
    Serial.print("Measured: ");
    Serial.print(T);*/
    //checks if reading is too far from the current average
    if(T - tempAverage > tempTolerance*measurementPeriod/(double)1000 || T - tempAverage < -1*tempTolerance*measurementPeriod/(double)1000)
    {
      //if it is, ignore the measurement
      //Serial.println(" Reject!");
      tempMisses++;
    }
    else
    {
      //otherwise update the average
      //Serial.println("  Accept.");
      tempMisses = 0;
      tempAverage += T/numTempMeasurements - temp[(tempMeasurement%numTempMeasurements)]/numTempMeasurements;
      temp[(tempMeasurement%numTempMeasurements)] = T;
      tempMeasurement++;
    }
    /*Serial.print("Array: ");
    for(int i = 0; i < numTempMeasurements; i++)
    {
      Serial.print(temp[i]);
      Serial.print("  ");
    }
    Serial.println();
    Serial.print("New Average: ");
    Serial.print(tempAverage);
    Serial.print("  ");
    Serial.print(tempMisses);
    Serial.println();*/


    //same for pressure
    double P = barometer.readPressure();
    /*Serial.println("Pressure:");
    Serial.print("Old Average: ");
    Serial.println(pressureAverage);
    Serial.print("Measured: ");
    Serial.print(P);*/
    if(P - pressureAverage > pressureTolerance*measurementPeriod/(double)1000 || P - pressureAverage < -1*pressureTolerance*measurementPeriod/(double)1000)
    {
      //Serial.println(" Reject!");
      pressureMisses++;
    }
    else
    {
      //Serial.println("  Accept.");
      pressureMisses = 0;
      pressureAverage += P/numPressureMeasurements - pressure[(pressureMeasurement%numPressureMeasurements)]/numPressureMeasurements;
      pressure[(pressureMeasurement%numPressureMeasurements)] = P;
      pressureMeasurement++;
    }
    /*Serial.print("Array: ");
    for(int i = 0; i < numPressureMeasurements; i++)
    {
      Serial.print(pressure[i]);
      Serial.print("  ");
    }
    Serial.println();
    Serial.print("New Average: ");
    Serial.print(pressureAverage);
    Serial.print("  ");
    Serial.print(pressureMisses);
    Serial.println("\n");*/


    //if there have been too many misreadings for either, call the function to reset the readings for it
    if(tempMisses > tempMaxMisses)
      reInitTemp();
    if(pressureMisses > pressureMaxMisses)
      reInitPressure();

    //increment the timer to dictate time for next measurement
    timer++;
  }
}
