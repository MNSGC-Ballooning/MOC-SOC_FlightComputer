#include <Smart.h>
#include <NMEAGPS.h>
#include <SoftwareSerial.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <Relay_XBee.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define smartPin 5//ENTER PIN HERE
#define redLED 3
#define greenLED 4
#define chipSelect 10 //SD logging
#define oneWireBus 11 //Dallas Temp Sensor
#define relayPin 7 //Grounding Pin 7 puts the Xbee into relay mode
#define xBee_Serial Serial //Xbee connected through Arduino SD shield
#define gps_Serial Serial1 //Ublox GPS

float cutTime = 75; //This is the time in minutes that we want before it cuts
float timer_smart = 1;
bool released = false;
NMEAGPS gps; 
//Below are different variables defined for the GPS
byte gpsMonth = 0;
byte gpsDay = 0;
byte gpsYear = 0;
byte gpsHours = 0;
byte gpsMinutes = 0;
byte gpsSeconds = 0;
float gpsLatitude = 0;
float gpsLongitude = 0;
float gpsAltitude = 0;
byte gpsSatellites = 0;

String PPOD;
String Command;
String ID = "SOC"; //This is the unique ID for the Xbee

//This is a counter for how many times in a row the PPOD tells the cube that it is open
float openhits = 0;

Smart smart = Smart(smartPin);
OneWire oneWire = OneWire(oneWireBus);
DallasTemperature sensors = DallasTemperature(&oneWire);
DeviceAddress inThermometer, outThermometer;
float inTemp, outTemp;
XBee xBee = XBee(&xBee_Serial, ID);

File datalog;                     //File object for datalogging
char filename[] = "GPSLOG00.csv"; //Template for file name to save data
bool SDactive = false;            //used to check for SD card before attempting to log

unsigned long timer_data = 0;        //used to keep track of datalog cycles
bool isOn = false;
bool relay = false;

// Geiger Counter:
long unsigned int timer_geiger = 0;  // Establish a variable to serve as a timer (counts milliseconds elapsed)
// unsigned int Logtime = 5000; // 5000 milliseconds (5 seconds) logging time for flight
unsigned int Logtime = 1000; // 1000 milliseconds (1 second) logging time for bench testing
long unsigned int LocalTime = 0;
long unsigned int LoopLog = 150;
long int counter1 = 0; // One-cycle counter for Geiger counter #1 hits (code can be expanded to handle multiple Geiger counters
long int totalcount1 = 0; // Cumulative Counter for Geiger counter #1 hits
boolean hit1 = 0; // Tells if sensor 1 was low during the current While loop: 1 if true, 0 if false
long int tempcounter = 0;  // Temporary counter variable

Adafruit_INA219 ina219;

void setup() {
  smart.initialize();
  Serial3.begin(115200); //This is the serial port the radio is connected to
  sensors.begin(); //This is the temp sensor
  gps_Serial.begin(9600); //Starting GPS communication

    if (!sensors.getAddress(inThermometer, 0)) {  //get address of first sensor; display error if not found
    Serial3.println("Unable to find address for Device 0");
  }

  sensors.setResolution(inThermometer, 9);  //set resolution for both sensors to 9 bits

  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(10, OUTPUT);    //This always needs to be an output when using SD

  Serial3.print("Initializing SD card...");
  if(!SD.begin(chipSelect)) {                               //attempt to start SD communication
    Serial3.println("Card failed, or not present");          //print out error if failed; remind user to check card
    for (byte i = 0; i < 10; i++) {                         //also flash error LED rapidly for 2 seconds, then leave it on
      digitalWrite(redLED, HIGH);
      delay(100);
      digitalWrite(redLED, LOW);
      delay(100);
    }
    digitalWrite(redLED, HIGH);
  }
  else {                                                    //if successful, attempt to create file
    Serial3.println("Card initialized.\nCreating File...");
    for (byte i = 0; i < 100; i++) {                        //can create up to 100 files with similar names, but numbered differently
      filename[6] = '0' + i/10;
      filename[7] = '0' + i%10;
      if (!SD.exists(filename)) {                           //if a given filename doesn't exist, it's available
        datalog = SD.open(filename, FILE_WRITE);            //create file with that name
        SDactive = true;                                    //activate SD logging since file creation was successful
        Serial3.println("Logging to: " + String(filename));  //Tell user which file contains the data for this run of the program
        break;                                              //Exit the for loop now that we have a file
      }
    }
    if (!SDactive) {
      Serial3.println("No available file names; clear SD card to enable logging");
      for (byte i = 0; i < 4; i++) {                        //flash LED more slowly if error is too many files (unlikely to happen)
        digitalWrite(redLED, HIGH);
        delay(250);
        digitalWrite(redLED, LOW);
        delay(250);
      }
      digitalWrite(redLED, HIGH);
    }
  }
//This defines the header variable and prints it over the radio and to the SD card
  String header = "MOC-SOC Data:, GPS Date,GPS time,Lat,Lon,Alt (m),# Sats, Temp (C), Cycle Hit Count, Cumulative Hit Count, Voltage (V), Time Left (min), PPOD:, Time Left (min), Distance Remaining (m)";  //setup data format, and print it to monitor and SD card
  Serial3.println(header);
  if (SDactive) {
    datalog.println("Time (min)," + header);
    datalog.close();
  }

//This starts the Xbee communication and starts it with the correct settings
  pinMode(relayPin, INPUT_PULLUP);
  xBee.initialize();
  if (digitalRead(relayPin) == LOW) {
    relay = true;
    xBee.enterATmode();
    xBee.atCommand("ATDL1");
    xBee.atCommand("ATMY0");
    xBee.exitATmode();

    xBee_Serial.begin(9600);
    Serial3.println("Relay Mode active");
  }

  //INA219
  uint32_t currentFrequency;
  ina219.begin();
  
}

void loop() {
  //INA219
  float loadvoltage = 0;
  float shuntvoltage = 0;
  float busvoltage = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  //Geiger counter
    // Check for sensor hits. Increment a counter if you get hits
  timer_geiger = millis();  // Record the original time
  
  while((millis()-timer_geiger) < Logtime)  // Check if Logtime (cycle time) has elapsed yet - cycle till it does
  {
    LocalTime = micros();
    int sensor1 = digitalRead(13); // Read in the value of pin 4 monitoring Geiger counter #1 - will be high except after a hit
    if(sensor1==LOW)
    {
      counter1++;  // Increment cycle counter
      totalcount1++;  // Increment cumulative counter
      hit1 = true;  // Note counter in "hit" state - repeat as long as counter stays low
    }

    hit1 = false;  // Once counter has recovered, reset it to "not hit" state

    //while((micros()-LocalTime)<LoopLog)
    //{
      //delayMicroseconds(5);  //slow code down (if needed) to let Geiger counter reset
    //}
  }
  
  if (!released && (millis()/60000.0 > cutTime)) {
    smart.release();

    released = true;
  }

  if (millis()/60000.0 > timer_smart) {
    timer_smart += 1;
    float TRemain = cutTime - millis()/60000.0;
    String Remain = "Auto: Time remaining is " + String(TRemain) + "minutes.";
  }

  if (gps.available( gps_Serial )){ //Starts reading and parsing GPS data
     gpsUpdate( gps.read() );
  }
  if (xBee_Serial.available() > 0) {
    String beacon = xBee_Serial.readString();
    //if (beacon.startsWith("Auto")) {
      Serial3.println(beacon);
    //}
    //else {
      //return;
    //}
  }
  
  if (millis() - timer_data > 30000) {  //log data once per second. This is a more accurate method than delay(), as it includes
    timer_data = millis();             //time taken for the program to execute, but requires a bit of setup
    

    sensors.requestTemperatures();                              //read most recent temp data for all sensors
    inTemp = sensors.getTempC(inThermometer);                   //get values from each sensor

    //All data is returned as numbers (int or float as appropriate), so values must be converted to strings before logging
    String data = String("MOC-SOC Data:") + "," + String(gpsMonth) + "/" + String(gpsDay) + "/" + String(gpsYear) + ","
                  + String(gpsHours) + ":" + String(gpsMinutes) + ":" + String(gpsSeconds) + ","
                  + String(gpsLatitude, 4) + "," + String(gpsLongitude, 4) + "," + String(gpsAltitude) + ","
                  + String(gpsSatellites) + "," + String(inTemp) + "," + String(counter1) + "," + String(totalcount1) + ","
                  + String(loadvoltage);
    if (released == false) {
      data = data + "," + String(cutTime - millis()/60000);  
    }
    else if (released == true) {
      data = data + "," + "Open";
    }
    digitalWrite(greenLED, HIGH); //flash green LED briefly when writing data
    Serial3.print(data);

    // Reset short-term (cycle) counter but not cumulative counter
    counter1 = 0;
    
    digitalWrite(greenLED, LOW);
     
     xBee_Serial.print("PPOD"); //This sends a command over Xbee to the PPOD to request data
     delay(5000); //Waits to give time for a reply
     if (xBee_Serial.available() > 0) {  //Checks to see if there is info in the Serial buffer
       String PPOD = xBee_Serial.readString();  //Reads said information
       Serial3.print(PPOD);  //Prints it to the radio

       if (PPOD == ",PPOD:, Released") {
        openhits += 1;
       }
       else {
        openhits = 0;
       }
     }
     xBee_Serial.print("ALT" + String(gpsAltitude));

  if (openhits >= 6) {
    smart.release();
    released = true;
  }
  
//Writes data to the SD card
  Serial3.println();

  datalog = SD.open(filename, FILE_WRITE);
  datalog.print(String(millis()/60000) + ",");
  datalog.print(data);
  datalog.print(PPOD);
  datalog.println();
  datalog.close();
  }

  //This section describes the commmands that can be sent from the ground to request information
  if (Serial3.available() > 0) {
    Command = Serial3.readString();
    Serial3.print(" This is what you typed: " + Command + ",");
    if (SDactive) {
    datalog = SD.open(filename, FILE_WRITE);
    datalog.print("," + String(Command));
    datalog.print(" This is what you typed: " + Command + ",");
    datalog.close();
  }

    if (Command == "Temp" || Command == "temp") {
      Serial3.println(",,,,,," + String(inTemp));
      datalog = SD.open(filename, FILE_WRITE);
      datalog.println(",,,,,," + String(inTemp));
      datalog.close();
    }
    else if (Command == "GPS" || Command == "gps") {
      Serial3.print(",," + String(gpsLatitude) + "," );
      Serial3.print(String(gpsLongitude, 4) + "," );
      Serial3.println(String(gpsAltitude, 4));
      datalog = SD.open(filename, FILE_WRITE);
      datalog.println(",," + String(gpsLatitude) + "," + String(gpsLongitude, 4) + "," + String(gpsAltitude, 4));
      datalog.close();
    }

    else if (Command == "geiger") {
      Serial3.println(",,,,,,," + String(counter1) + "," + String(totalcount1));
      datalog = SD.open(filename, FILE_WRITE);
      datalog.println(",,,,,,," + String(counter1) + "," + String(totalcount1));
      datalog.close();
    }

    else if (Command == "volt") {
      Serial3.println(",,,,,,,,," + String(loadvoltage));
      datalog = SD.open(filename, FILE_WRITE);
      datalog.println(",,,,,,,,," + String(loadvoltage));
      datalog.close();
    }

    else if (Command == "open") {
      Serial3.println("Opening Panels");
      smart.release();
      released = true;
    }

    else if (Command.startsWith("+")) {
      Command.remove(0,1);
      float addtime = Command.toFloat(); 
      cutTime = cutTime + addtime;
      Serial3.print(String(addtime) + " Minutes added." + " New Time Left: ," + String(cutTime - millis()/60000));
    }
    
    else if (Command.startsWith("-")) {
      Command.remove(0,1);
      float subtime = Command.toFloat();
      cutTime = cutTime - subtime;
      Serial3.print(String(subtime) + " Minutes removed." + " New Time Left: ," + String(cutTime - millis()/60000));
    }

    else if (Command == "hello") {
      Serial3.print("Still Here");
    }

    else if (Command.startsWith("P")) {
      Command.remove(0,1);
      xBee_Serial.print(Command); //This sends a command over Xbee to the PPOD to send command
      delay(5000); //Waits to give time for a reply
      if (xBee_Serial.available() > 0) {  //Checks to see if there is info in the Serial buffer
       String Response = xBee_Serial.readString();  //Reads said information
       Serial3.print(Response);  //Prints it to the radio
     }
    }
   
    else if (!Command.equals("")) {
      Serial3.println("Command Not Recognized - Please input valid command");
      datalog = SD.open(filename, FILE_WRITE);
      datalog.println("Command Not Recognized - Please input valid command");
      datalog.close();
    }

  }
  
}

//This is a function that the main loop calls to request new data from the GPS
void gpsUpdate( const gps_fix & fix ){

  if (fix.valid.date & fix.valid.time & fix.valid.location) {
 gpsMonth=fix.dateTime.month;
 gpsDay=fix.dateTime.date; 
 gpsYear=fix.dateTime.year;
 gpsHours=fix.dateTime.hours - 5; //The time is set to UST which we are 5 hours behind 
 gpsMinutes=fix.dateTime.minutes;
 gpsSeconds=fix.dateTime.seconds;
 gpsLatitude=(fix.latitude());
 gpsLongitude=(fix.longitude());
 gpsAltitude=(fix.altitude());
 gpsSatellites=fix.satellites;

  }
  else {
    gpsMonth=0;
    gpsDay=0;
    gpsYear=0;
    gpsHours=0;
    gpsMinutes=0;
    gpsSeconds=0;
    gpsLatitude=0;
    gpsLongitude=0;
    gpsAltitude=0;
    gpsSatellites=0;

}
}
