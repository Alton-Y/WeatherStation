/*
GUSTAV Weather Station
Original By:  Nathan Paes
GPS from tinygps
*/

#include <Wire.h> //I2C needed for sensors
#include "SparkFunMPL3115A2.h" //Pressure sensor - Search "SparkFun MPL3115" and install from Library Manager
#include "SparkFunHTU21D.h" //Humidity sensor - Search "SparkFun HTU21D" and install from Library Manager
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>


//GPS
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
TinyGPS gps;
SoftwareSerial mySerial(5, 4);
// static void print_date(TinyGPS &gps);
static void smartdelay(unsigned long ms);
//static void print_int(unsigned long val, int len);
//static void print_float(float val, int len, int prec);

MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor

//SD
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
File logfile;
const int chipSelect = 10;  // SD chip select for sheild

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
int WSPEED = 2;
int STAT1 = 7;
int STAT2 = 8;

// analog I/O pins
int REFERENCE_3V3 = A3;
int LIGHT = A1;
int BATT = A2;
int WDIR = A0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned long weathertimer;
unsigned long flushtimer;
unsigned long gpstimer;

volatile float Time = 0; //wind data
volatile float deltaTime = 0; //wind data




void setup()
{
  digitalWrite(STAT1, HIGH); //Blink stat LED
  digitalWrite(STAT2, HIGH); //Blink stat LED
  Serial.begin(9600);
  //  Serial.println("\n+---------------+       +-----------------------------------------+");
  //  Serial.println("| RYERSON       |       | GUSTAV Weather Station                    |");
  //  Serial.println("| APPLIED       |       | https://github.com/Alton-Y/WeatherStation |");
  //  Serial.println("| AERODYNAMICS  |       |                                           |");
  //  Serial.println("| LABORATORY OF |       |                                           |");
  //  Serial.println("| FLIGHT        |       |                                           |");
  //  Serial.println("+---------------+       +-------------------------------------------+\n");

  Serial.println("\n+---------------+");
  Serial.println("| RYERSON       |");
  Serial.println("| APPLIED       |");
  Serial.println("| AERODYNAMICS  |");
  Serial.println("| LABORATORY OF |");
  Serial.println("| FLIGHT        |");
  Serial.println("+---------------+\n");

  pinMode(STAT1, OUTPUT); //Status LED Blue
  pinMode(STAT2, OUTPUT); //Status LED Green

  pinMode(WDIR, INPUT);
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  //pinMode(WSPEED, INPUT);
  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  //Configure the pressure sensor
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags
  //
  //Configure the humidity sensor
  myHumidity.begin();


  // attach external interrupt pins to IRQ functions (wind speed time delta)
  attachInterrupt(0, delta, FALLING);



  //SD
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
  }
  else {
    Serial.println("card initialized.");
  }

  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      Serial.println("file open.");
      break;  // leave the loop!
    }
  }

  Serial.println("Weather Shield online!");

  //GPS
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  Serial.print("Initializing GPS...");
  mySerial.begin(9600);

  delay(1000);

  Serial.println("GPS initialized.");

  //Headers in Excel file
  logfile.println(F("boardtime,winddir,windspeed,humidity,tempf,pressure,batt_lvl,light,gpsfix,numsats,hdop,age,year,month,day,hour,min,second,hundredth,lat,lon"));
  Serial.println(F("boardtime,winddir,windspeed,humidity,tempf,pressure,batt_lvl,light,gpsfix,numsats,hdop,age,year,month,day,hour,min,second,hundredth,lat,lon"));

  //Start timers and end setup
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  weathertimer = millis();
  flushtimer = millis();
  gpstimer = millis();

  digitalWrite(STAT1, LOW); //Blink stat LED
  digitalWrite(STAT2, LOW); //Blink stat LED
}



void loop()
{

  //Keep track of which minute it is

  if (millis() - weathertimer >= 500) {

    float humidity = 0; // [%]
    float tempf = 0; // [%]
    float pressure = 0;
    float batt_lvl = 11.8; //[analog value from 0 to 1023]
    float light_lvl = 455; //[analog value from 0 to 1023]
    int winddir = 0; // [0-360 instantaneous wind direction]
    float currentSpeed = 0;

    weathertimer = millis();
    digitalWrite(STAT1, HIGH); //Blink stat LED

    calcWeather(winddir, currentSpeed, humidity, tempf, pressure, batt_lvl, light_lvl); //Go calc all the various sensors

    printWeather(winddir, currentSpeed, humidity, tempf, pressure, batt_lvl, light_lvl); //record data

    digitalWrite(STAT1, LOW); //Turn off stat LED
  }



  //GPS
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  if (millis() - gpstimer >= 15000) {
    digitalWrite(STAT1, HIGH); //Blink stat LED
    gpstimer = millis();

    float flat, flon;
    unsigned long age, date, time, chars = 0;
    int year , fix;
    byte month, day, hour, minute, second, hundredths;

    smartdelay(1000); //tinygps waits for 1 second to read 1Hz input from GPS
    calcGPS(flat, flon, age, date, time, chars, year, fix, month, day, hour, minute, second, hundredths);

    if (fix == 0) {
      digitalWrite(STAT2, LOW);
      printemptyGPS();
    }
    else {
      digitalWrite(STAT2, HIGH);
      printGPS(flat, flon, age, date, time, chars, year, fix, month, day, hour, minute, second, hundredths);
    }
    digitalWrite(STAT1, LOW); //Blink stat LED
  }

  delay(50);

  if (millis() - flushtimer >= 1000) { //flush SD buffer
    flushtimer = millis();
    logfile.flush();
  }
  // if timer wraps around, reset
  //if (weathertimer > millis())  weathertimer = millis();
  //if (flushtimer > millis())  flushtimer = millis();
  //if (gpstimer > millis())  gpstimer = millis();
}

//Calculates each of the variables from weather shield
void calcWeather(int &winddir, float &currentSpeed, float &humidity, float &tempf, float &pressure, float &batt_lvl, float &light_lvl) //addresses are passed in, not values
{
  //Calc winddir
  winddir = get_wind_direction();

  //Calc the wind speed
  currentSpeed = get_wind_speed();

  //Calc humidity
  humidity = myHumidity.readHumidity();

  //Calc tempf from pressure sensor
  tempf = myPressure.readTempF();

  //Calc pressure
  pressure = myPressure.readPressure();

  //Calc light level
  light_lvl = get_light_level();

  //Calc battery level
  batt_lvl = get_battery_level();

}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)

//New version with Alton's irradiance meter on A1
float get_light_level()
{
  //float operatingVoltage = analogRead(REFERENCE_3V3);

  float lightSensor = analogRead(LIGHT);

  //operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

  //lightSensor = operatingVoltage * lightSensor;

  return (lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  delay(1);
  float rawVoltage = analogRead(BATT);
  delay(1);
  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V

  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin

  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

  return (rawVoltage);
}

void delta()
{
  Time = (micros() - deltaTime) / 1000000;  // check difference time from last rotation
  deltaTime = micros();           //set the old time to the current time for the next iteration

}
// Get Wind Speed

float get_wind_speed()
{
  float WindSpeed; // speed miles per hour
  if (Time == 0 || (micros() - deltaTime) > 8000000) { //8 second timeout
    WindSpeed = 0;
  }
  else {
    WindSpeed = 1.00584 / Time;   // get wind speed in m/s    // Davis formula V=P*(2.25/T)  where P =#of Rotations, T= sample period in seconds
    //Set P=1, count to find T then multiply by 0.44704 to convert from mph to m/s
  }
  if (WindSpeed > 100)
  {
    WindSpeed = -9999;
  }
  //reset Time to 0

  return (WindSpeed);

}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  int dir[20];
  for (int x = 0; x < 20; x++) { //average 20 subsequent readings
    dir[x] = analogRead(WDIR);
    delay(2);
  }

  float ave = average(dir, 20);
  //Basic potentiometer, maps 0->1023 to 0->360
  //return(dir);
  return (map(ave, 0, 1023, 0, 360));


}


//Prints the various variables directly to the port

void printWeather(int winddir, float currentSpeed, float humidity, float tempf, float pressure, float batt_lvl, float light_lvl) //values are passed in, not addresses
{
  logfile.print(millis());
  logfile.print(",");
  logfile.print(winddir);
  logfile.print(",");
  logfile.print(currentSpeed, 5);
  logfile.print(",");
  logfile.print(humidity);
  logfile.print(",");
  logfile.print(tempf, 9);
  logfile.print(",");
  logfile.print(pressure);
  logfile.print(",");
  logfile.print(batt_lvl);
  logfile.print(",");
  logfile.print(light_lvl);
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.println(",");


  Serial.print(millis());
  Serial.print(",");
  Serial.print(winddir);
  Serial.print(",");
  Serial.print(currentSpeed, 5);
  Serial.print(",");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(tempf, 9);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(batt_lvl);
  Serial.print(",");
  Serial.print(light_lvl);
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.println(",");

  //  Serial.print("winddir=");
  //  Serial.print(winddir);
  //  Serial.print(",windspeedm/s=");
  //  Serial.print(currentSpeed, 1);
  //  Serial.print(",humidity=");
  //  Serial.print(humidity, 1);
  //  Serial.print(",tempf=");
  //  Serial.print(tempf, 1);
  //  Serial.print(",pressure=");
  //  Serial.print(pressure, 2);
  //  Serial.print(",batt_lvl=");
  //  Serial.print(batt_lvl, 2);
  //  Serial.print(",light_lvl=");
  //  Serial.print(light_lvl, 2);
  //  Serial.println("");

}

void calcGPS(float &flat, float &flon, unsigned long &age,  unsigned long &date,  unsigned long &time,  unsigned long &chars, int &year, int &fix, byte &month, byte &day, byte &hour, byte &minute, byte &second, byte &hundredths)
{
  gps.f_get_position(&flat, &flon, &age);
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

  if (age == TinyGPS::GPS_INVALID_AGE) {
    fix = 0;
  }
  else  {
    fix = 1;
  }
}

void printGPS(float flat, float flon, unsigned long age,  unsigned long date,  unsigned long time,  unsigned long chars, int year, int fix, byte month, byte day, byte hour, byte minute, byte second, byte hundredths)
{
  logfile.print(millis());
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(fix);
  logfile.print(",");
  logfile.print(gps.satellites());
  logfile.print(",");
  logfile.print(float(gps.hdop()) / 100.00);
  logfile.print(",");
  logfile.print(age);
  logfile.print(",");
  logfile.print(year);
  logfile.print(",");
  logfile.print(month);
  logfile.print(",");
  logfile.print(day);
  logfile.print(",");
  logfile.print(hour);
  logfile.print(",");
  logfile.print(minute);
  logfile.print(",");
  logfile.print(second);
  logfile.print(",");
  logfile.print(hundredths);
  logfile.print(",");
  logfile.print(flat, 9);
  logfile.print(",");
  logfile.println(flon, 9);

  Serial.print(millis());
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(fix);
  Serial.print(",");
  Serial.print(gps.satellites());
  Serial.print(",");
  Serial.print(float(gps.hdop()) / 100.00);
  Serial.print(",");
  Serial.print(age);
  Serial.print(",");
  Serial.print(year);
  Serial.print(",");
  Serial.print(month);
  Serial.print(",");
  Serial.print(day);
  Serial.print(",");
  Serial.print(hour);
  Serial.print(",");
  Serial.print(minute);
  Serial.print(",");
  Serial.print(second);
  Serial.print(",");
  Serial.print(hundredths);
  Serial.print(",");
  Serial.print(flat, 9);
  Serial.print(",");
  Serial.println(flon, 9);
  //  Serial.print("fix=");
  //  Serial.print(fix);
  //  Serial.print(", numSats=");
  //  Serial.print(gps.satellites());
  //  Serial.print(", hdop=");
  //  Serial.print(gps.hdop());
  //  Serial.print(", age=");
  //  Serial.print(age);
  //  Serial.print(", year=");
  //  Serial.print(year);
  //  Serial.print(", month=");
  //  Serial.print(month);
  //  Serial.print(", day=");
  //  Serial.print(day);
  //  Serial.print(", hour=");
  //  Serial.print(hour);
  //  Serial.print(", minute=");
  //  Serial.print(minute);
  //  Serial.print(", second=");
  //  Serial.print(second);
  //  Serial.print(", hundredths=");
  //  Serial.print(hundredths);
  //  Serial.print(", lat=");
  //  Serial.print(flat);
  //  Serial.print(", lon=");
  //  Serial.print(flon);
  //  Serial.println();

}

void printemptyGPS()
{

  logfile.print(millis());
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print("0");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.print(",");
  logfile.println(",");

  Serial.print(millis());
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print("0");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.print(",");
  Serial.println(",");
  //  Serial.println();
  //  Serial.print("fix=");
  //  Serial.print("0");
  //  Serial.print(", numSats=");
  //  Serial.print(" ");
  //  Serial.print(", hdop=");
  //  Serial.print(" ");
  //  Serial.print(", lat=");
  //  Serial.print(" ");
  //  Serial.print(", lon=");
  //  Serial.print(" ");
  //  Serial.print(", age=");
  //  Serial.print(" ");
  //  Serial.print(", year=");
  //  Serial.print(" ");
  //  Serial.print(", month=");
  //  Serial.print(" ");
  //  Serial.print(", day=");
  //  Serial.print(" ");
  //  Serial.print(", hour=");
  //  Serial.print(" ");
  //  Serial.print(", minute=");
  //  Serial.print(" ");
  //  Serial.print(", second=");
  //  Serial.print(" ");
  //  Serial.print(", hundredths=");
  //  Serial.print(" ");;
  //  Serial.println();

}

float average (int * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}


static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (mySerial.available())
      gps.encode(mySerial.read());
  } while (millis() - start < ms);
}

//static void print_date(TinyGPS &gps)
//{
//  int year;
//  byte month, day, hour, minute, second, hundredths;
//  unsigned long age;
//  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
//  if (age == TinyGPS::GPS_INVALID_AGE)
//    Serial.print("********** ******** ");
//  else
//  {
//    char sz[32];
//    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
//        month, day, year, hour, minute, second);
//    Serial.print(sz);
//  }
//  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
//  smartdelay(0);
//}

//static void print_int(unsigned long val, int len)
//{
//  char sz[32];
//  sprintf(sz, "%ld", val);
//  sz[len] = 0;
//  for (int i=strlen(sz); i<len; ++i)
//    sz[i] = ' ';
//  if (len > 0)
//    sz[len-1] = ' ';
//  Serial.print(sz);
//}

//static void print_float(float val, int prec)
//{
//    Serial.print(val, prec);
//    int vi = abs((int)val);
//    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
//    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
//}
