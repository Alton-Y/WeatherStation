// Master data input arduino
// By: Nathan Paes
//5Hz update rate

#include <Adafruit_GPS.h> //for gps
#include <SoftwareSerial.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h" //Pressure sensor 
#include "SparkFunHTU21D.h" //Humidity sensor 

#define GPSECHO  false // no echo


MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor
SoftwareSerial mySerial(3, 2); //2 to RX on GPS and 3 to TX on GPS
Adafruit_GPS GPS(&mySerial) ;// 'attach' GPS to pins 3, and 2
boolean usingInterrupt = false;
void useInterrupt(boolean);
byte pressureParts[2];
byte tempParts[2];
byte GPSmilli[2];

//********************************************SETUP*************************************************************
void setup() {
  Wire.begin(8); // join i2c bus (address optional for master)
  Serial.begin(9600);

  //initalize GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 5 Hz update rate
  useInterrupt(true);

  //Configure theweather sheild
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

  //Configure the humidity sensor
  myHumidity.begin();
  delay(8000); //give slave arduino time to setup SD card and file  Note this should not affect sync rate since it occours once during setup

}
//**************************************************************************************************************

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif

}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
//********************************************MAIN LOOP*************************************************************
void loop() {         //No delays in loop pls kthx


  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  //read humidity
  float humidity = myHumidity.readHumidity();

  if (humidity == 998) //Humidty sensor failed to respond
  {
    Serial.println(F("I2C communication to sensors is not working. Check solder connections."));

    //Try re-initializing the I2C comm and the sensors
    myPressure.begin();
    myPressure.setModeBarometer();
    myPressure.setOversampleRate(7);
    myPressure.enableEventFlags();
    myHumidity.begin();
  }
  else //Succesfull
  {


    byte humid = (byte)humidity;    //convert humidity to an integer number
    //   Serial.print(humid); //For Debugging
    //    Serial.print(":");
    //    Serial.println(humid);


    float temp_h = myHumidity.readTemperature();
    //int AmbientT = (int)(temp_h);       // convert float number to a rounded integer
    //    Serial.print("\t\t");
    //Serial.println(AmbientT); //For Debugging

    //Check Pressure Sensor
    float pressure = myPressure.readPressure();       //**change all to bytes
    Serial.print(temp_h);
    Serial.print("\t");

    //    int PAtm1 = (int)(pressure / 1000);       //since pressure is usually 5 t 6 digits we split it up into two parts so the slave arduino can receive it in two parts and then stitch it togather again
    //    pressure = pressure - 1000 * PAtm1;
    //    int PAtm3 = (int)(pressure);
    //    int PAtm2 = (int)(PAtm3 / 10);
    //    PAtm3 = PAtm3 - PAtm2 * 10;
    //IntToByte(temp_h, tempParts, 1);
    tempParts[1]=byte(temp_h);
    tempParts[2]=byte(temp_h*100-tempParts[1]*100);
     Serial.print(tempParts[1]);
     Serial.print("\t");
     Serial.println(tempParts[2]);
    IntToByte(pressure, pressureParts, 1000);
    IntToByte(GPS.milliseconds, GPSmilli, 1);
    byte latitude = (byte)(GPS.latitude, 1);
    byte longitude = (byte)(GPS.longitude, 1);
    byte alt = (byte)(GPS.altitude);
    //**********************************************************************TRANSMIT TO SLAVE
    Wire.beginTransmission(8); // transmit to device #8
    Wire.write(latitude);      //1
    Wire.write(longitude);    //2
    Wire.write(alt);          //3
    Wire.write(GPS.month);    //4
    Wire.write(GPS.day);      //5
    Wire.write(GPS.year);     //6
    Wire.write(GPS.hour);     //7
    Wire.write(GPS.minute);   //8
    Wire.write(GPS.seconds);  //9
    Wire.write(GPSmilli[0]);//10
    Wire.write(GPSmilli[1]);//11
    Wire.write(humid);        //12
    Wire.write(tempParts[1]);     //13
    Wire.write(tempParts[2]);     //14
    Wire.write(pressureParts[0]);        //15
    Wire.write(pressureParts[1]);        //16
    Wire.write(pressureParts[2]);        //17
    Wire.endTransmission();    // stop transmitting
  }
  //**************************************************************************************
}
//******************************************************************************************************************
void IntToByte(long x, byte b[], int factor)
{
  b[0] = (byte)(x / factor);
  x = x - ((byte)(x / factor)) * factor;
  b[1] = (byte)(x / factor * 100);
  x = x - ((byte)(x / factor * 100)) * factor * 100;
  b[2] = (byte)(x);
}

