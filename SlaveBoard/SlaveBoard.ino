// Slave datalogging arduino
// By: Nathan Paes
//5Hz log rate

#include <Wire.h>
//*************for logging
#include <SPI.h>
#include <SD.h>
//#include "RTClib.h"

//Anemometer
//#include "TimerOne.h" // Timer Interrupt set to 2 second for read sensors 
#include <math.h>
#include <LiquidCrystal.h>

#define LOG_INTERVAL 100 //5hz rate 
#define ECHO_TO_SERIAL   1  // echo data to serial port
#define WAIT_TO_START    0  // Wait for serial input in setup()
#define WindSensorPin (2)   // The pin location of the anemometer sensor you have to use 2 or 3
#define WindVanePin (A0)    // The pin the wind vane sensor is connected to 
//#define VaneOffset 0;     // define the anemometer offset from magnetic north

int VaneValue; // raw analog value from wind vane
int Direction; // translated 0 - 360 direction
int CalDirection; // converted value with offset applied
int LastValue; // last direction value
const int button = 7;
//const int GreenLed = 8;
//const int RedLed = 9;
uint32_t LastSync = millis();

//volatile bool IsSampleRequired; // this is set true every 2.5s. Get wind speed
//volatile unsigned int TimerCount; // used to determine 2.5sec timer count
//volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
//volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in isr
volatile float Time;
volatile float deltaTime;
float WindSpeed; // speed miles per hour
LiquidCrystal lcd( 8, 9, 6, 4, 5, 3); //9,8,5,4,3,6
const int chipSelect = 10;  // SD chip select for sheild
File logfile;
//*************************For logging
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  lcd.setCursor(10, 1);
  lcd.print("ERROR  ");
  lcd.display();
  while (1);
}
//************************************

//********************************************SETUP*************************************************************
void setup() {
  Serial.begin(9600);           // start serial for output
  //For LCD
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("  RAALF Weather");
  lcd.setCursor(0, 1);
  lcd.print("    Station   :)");
  lcd.display();
  delay(1000);
  lcd.clear();  
  pinMode(button, OUTPUT);
  digitalWrite(button, LOW);
  pinMode(button, INPUT);
  
  //*************************************For logging
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to output, even if you don't use it
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  // digitalWrite(GreenLed, LOW);
  // create a new file
  char filename[] = "LOG00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = i / 10 + '0';
    filename[4] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;
    }
  }

  if (! logfile) {
    error("couldnt create file");
  }

  Serial.print("Logging to: ");
  Serial.println(filename);
  lcd.setCursor(10, 1);
  lcd.print(filename);
  lcd.display();
  //**********************************************
  //Headers in Excell file
  logfile.println(F("Lattitude (Deg),Longitude (Deg),Altitude,Month,Day,Year,Hour,Minute,Second,Millisecond,Humidity,Temp,Pressure,Wind Speed,Wind Direction,Millis"));
#if ECHO_TO_SERIAL
  Serial.println("Lattitude (Deg),Longitude (Deg),Altitude,Month,Day,Year,Hour,Minute,Second,Millisecond,Humidity,Temp,Pressure,Wind Speed,Wind Direction,Millis");
#endif //ECHO_TO_SERIAL
  delay(1000);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  
  pinMode(WindVanePin, OUTPUT);
  digitalWrite(WindVanePin, LOW);
  pinMode(WindVanePin, INPUT);
  pinMode(WindSensorPin, INPUT);
  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), delta, FALLING);      //When the value of pin 2 falls call the delta function
  Time = 1000000;        //large number so first speed value is 0
  deltaTime = 0;        //set to zero initally
}
//**************************************************************************************************************

void loop() { //useless

}

//********************************************MAIN LOOP*********************************************************
//Runs when data is received
void receiveEvent(int howMany) {      //Read constantly....

  //byte x1 = Wire.read();    // Latitude
  //byte x2 = Wire.read();    // Longitude
  //byte x3 = Wire.read();    // Altitude
  byte x4 = Wire.read();    // Month
  byte x5 = Wire.read();    // Day
  byte x6 = Wire.read();    // Year
  byte x7 = Wire.read();    // Hour
  byte x8 = Wire.read();    // Minute
  byte x9 = Wire.read();    // Second
  byte ms0 = Wire.read();   // Millisecond 1
  byte ms1 = Wire.read();   // Millisecond 2
  byte x11 = Wire.read();  // Humidity
  byte x12 = Wire.read();   // Temperature 1 
  byte x13 = Wire.read();   // Temperature 2 
  byte x14 = Wire.read();    // Pressure 1
  byte x15 = Wire.read();    // Pressure 2
  byte x16 = Wire.read();   // Pressure 3
  byte x17 = Wire.read();   // GPS SAT Count

  float temp = x12+(x13)/100.00;
  float pressure = x14*1000.00+x15*10.00+x16;
  float x10 = ms0*100.00+ms1;
  
  getWindSpeed();
  getWindDirection();
  if (abs(CalDirection - LastValue) > 5) {
    LastValue = CalDirection;
  }

  //********************LOG DATA
  //logfile.print(x1);   //Latitude
  //logfile.print(",");
  //logfile.print(x2);   //Longitude
  //logfile.print(",");
  //logfile.print(x3);   //Altitude
  //logfile.print(",");
  logfile.print(x4);    //Month
  logfile.print(",");
  logfile.print(x5);    //Day
  logfile.print(",");
  logfile.print(x6);    //Year
  logfile.print(",");
  logfile.print(x7);    //Hour
  logfile.print(",");
  logfile.print(x8);    //Minute
  logfile.print(",");
  logfile.print(x9);    //Second
  logfile.print(",");
  logfile.print(x10);    //Millisecond
  logfile.print(",");
  logfile.print(x17);    //SAT Count
  logfile.print(",");
  logfile.print(x11); //Humidity
  logfile.print(",");
  logfile.print(temp);  //Temperature 1
  logfile.print(",");
  logfile.print(pressure);  //Pressure in parts
  logfile.print(",");
  logfile.print(WindSpeed);
  logfile.print(",");
  logfile.print(Direction);
  logfile.print(",");
  logfile.println(millis());
#if ECHO_TO_SERIAL      //Print to screen so we know whats going on
  //Serial.print(x1);
  //Serial.print("\t\t");
  //Serial.print(x2);
  //Serial.print("\t\t");
  //Serial.print(x3);
  //Serial.print("\t");
  Serial.print(x4);
  Serial.print("/");
  Serial.print(x5);
  Serial.print("/");
  Serial.print(x6);
  Serial.print("  ");
  Serial.print(x7);
  Serial.print(":");
  Serial.print(x8);
  Serial.print(":");
  Serial.print(x9);
  Serial.print(":");
  Serial.print(x10);
  Serial.print(" ,\t SAT");
  Serial.print(x17);
  Serial.print(" ,\t");
  Serial.print(x11);
  Serial.print(" ,\t");
  Serial.print(temp);
  Serial.print(",\t");
  Serial.print(pressure);
  Serial.print(",\t");
  Serial.print(",    ");
  Serial.print(WindSpeed);
  Serial.print(",    ");
  Serial.print(Direction);
  Serial.print(",    ");
  Serial.println(millis());
#endif //ECHO_TO_SERIAL
  //******************************
  logfile.flush();//Sync to SD card
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("          ");
  lcd.setCursor(0, 0);
  lcd.print(WindSpeed);
  lcd.setCursor(6, 0);
  lcd.print(Direction);
  lcd.setCursor(9, 0);
  lcd.print((char)223);
  lcd.setCursor(12, 0);
  lcd.print(int(temp));
  lcd.setCursor(14, 0);
  lcd.print((char)223);
  lcd.setCursor(15, 0);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print(x7);
  lcd.setCursor(2, 1);
  lcd.print(":");
  lcd.setCursor(3, 1);
  lcd.print(x8);
  lcd.setCursor(5, 1);
  lcd.print(":");
  lcd.setCursor(6, 1);
  lcd.print(x9);
  lcd.display();

  endLog();   //Constantly check if we need to end the logging process
}
//**************************************************************************************************************

//Delta function to retrive windspeed
void delta()
{
  Time = (millis() - deltaTime) / 1000;  // check difference time from last rotation
  deltaTime = millis();           //set the old time to the current time for the next iteration
}
// Get Wind Direction

void getWindSpeed()
{
  if ((millis() - deltaTime) > 4000) ///if a rotation takes more than 10 seconds then just mark it as 0 essentially
  {
    Time = 1000000;     //large number so first speed value is 0
  }
  WindSpeed = 1.00584 / Time;   // get wind speed in m/s    // Davis formula V=P*(2.25/T)  where P =#of Rotations, T= sample period in seconds
  //Set P=1, count to find T then multiply by 0.44704 to convert from mph to m/s
  if (WindSpeed > 100)
  {
    WindSpeed = -9999;
  }


}
void getWindDirection() {       //Basic potentiometer, maps 0->1023 to 0->360

  VaneValue = analogRead(WindVanePin);
  Direction = map(VaneValue, 0, 1023, 0, 360);
  //CalDirection = Direction + VaneOffset;

  if (CalDirection > 360)
    CalDirection = CalDirection - 360;

  if (CalDirection < 0)
    CalDirection = CalDirection + 360;

}
//If button is pressed close the file then end the log
void endLog()
{
  int ButtonState = digitalRead(button);
  if (ButtonState == HIGH)
  {
    lcd.setCursor(9, 1);
    lcd.print("END    ");
    lcd.display();
    Serial.print(F("end of logging"));
    delay(1000);
    logfile.close();
    while (1);
  }
  else
    return;
}



