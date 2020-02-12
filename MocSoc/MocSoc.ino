//MOC-SOC Flight Computer
//Written by Paul Wehling with assistance from Lilia Bouayed, Based on code originally written by Jordan Bartlett and Emma Krieg.
//For use with MOC-SOC 3U Cubesat 1.1, built by Paul Wehling and Donald Rowell, based on original by Jordan Bartlett and Emma Krieg.
//Last updated February 12, 2020

  #include <Servo.h>
  #include <SD.h>
  #include <DallasTemperature.h>
  #include <OneWire.h>
  #include <Adafruit_INA219.h>
  #include <GeigerCounter.h>
  #include <RelayXBee.h>
  #include <UbloxGPS.h>

  #define chipSelect 53       //primary pin for SD Logging
  #define tempBus 11          //data pin for temperature probe
  #define smartPin 6          //digital pin for SMART release system
  #define rbfPin A0           //pin through which remove-before-flight pin connects to 5V rail
  #define timeTillDeploy 70   //flight time in minutes after which to deploy panels
  #define xbeeSerial Serial   //communication channel for shield XBee
  #define cycle_time_sec 10   //time in seconds for a complete logging cycle
  #define ubloxSerial Serial1 //communication channel for UBLOX GPS
  #define gpsTolerance 10     //ft/s the gps is allowed to drift from last contact
  #undef  abs                 //allows abs() to be used as a regular function instead of the macro that Arduino usually uses
  
  
  short cutTime = timeTillDeploy;                                 //flight time after which panels attempt deploy, can be changed in flight via SETT, +, or - commands
  File datalog;                                                   //File object for datalogging
  char data_filename[] = "MSDATA00.csv";                          //file name template for data file
  char report_filename[]= "MSRPRT00.dat";                         //file name template for report file (contains serial data)
  bool SD_data_active = false;                                    //checks if SD data logging is working
  bool SD_report_active = false;                                  //checks if SD report logging is working
  OneWire oneWire(tempBus);                                       //creates OneWire object for temperature sensor
  DallasTemperature sensors(&oneWire);                            //creates object for temperature measurement
  float temperature = 0.0;                                        //temperature variable
  Servo servo;                                                    //SMART object for panel deployment
  bool smart_release_attempt = false;                             //checks if smart.release() has been called
  bool flight_begun = false;                                      //checks if MocSoc has left ground
  unsigned long millis_start_time = 0;                            //records time at which launch occurs
  Adafruit_INA219 ina219;                                         //creates object for solar panel voltage measurement
  GeigerCounter geiger1 = GeigerCounter(2);                       //creates object for geiger counter (extra note: use enternal interrupt pins for geiger)
  String ID = "SOC";                                              //unique ID for XBee connection
  RelayXBee xBee = RelayXBee(&xbeeSerial, ID);                    //creates object for XBee communication
  unsigned long last_millis_overflow = 0;                         //creates object to hold start time of last cycle
  short cycle_time = cycle_time_sec * 1000;                       //time in seconds for a complete logging cycle, can be changed in flight via SETC command
  bool timestamp_hold = false;                                    //helps printout() keep timestamps at start of all data recordings
  String gps_lock;                                                //holds last gps coordinates, no filter applied
  short gps_alt[3];                                               //holds last 3 gps altitudes
  unsigned long gps_time[3];                                      //holds time of last three gps locks (0 starts at millis_start_time)
  UbloxGPS gps = UbloxGPS(&ubloxSerial);                          //creates object for GPS tracking
  bool gps_good = false;                                          //tracks if GPS readings have cleared filter
  String report_data;                                             //contains qualitative data and radio traffic receipts
  bool extra_data = false;                                        //tracks if there is extra data to print during a given cycle
  

void setup() {
  Serial.begin(9600);       //starts serial communication to computer serial moniter/XBEE
  Serial2.begin(115200);    //starts serial communication to freewave radio
  sdSetup();                //starts SD Card logging and creates logging files
  Serial1.begin(UBLOX_BAUD);//starts communication to UBLOX
  radioAndGpsSetup();       //begins radio communication through XBee and gps tracking through UBLOX
  sensorsSetup();           //activates temperature sensors, geiger counter, solar panels, and SMART system
}

void loop() {
  if(!flight_begun) checkStart();                                 //determines if flight has started yet based on remove-before-flight switch
  else if(!smart_release_attempt) checkDeploy();                  //activates SMART after cutTime elapses
  recieveCommands();                                              //checks XBee for commands from ground/comms/etc
  attemptGPS(false);                                              //tries to establish a gps lock and filters for good and bad hits
  printdata();                                                    //prints the data to radio, serial, SD, and XBEE
  wait();                                                         //holds until cycle time has elapsed
}

void sdSetup() {                 //starts SD Card logging and creates logging files
  //pinMode(53,OUTPUT);
  printout("Initializing SD card",true);
  if(!SD.begin(chipSelect)) printout("ERROR: Card failed, or not present",true);
  else {
    printout("Card initialized.\nCreating Files",true);
    for (byte i = 0; i < 100; i++) {
      report_filename[6] = '0' + i/10;
      report_filename[7] = '0' + i%10;
      if (!SD.exists(report_filename)) {
        datalog = SD.open(report_filename, FILE_WRITE);
        datalog.close();
        SD_report_active = true;
        printout("Logging report to: " + String(report_filename),true);
        break;
      }
    }
    for (byte i = 0; i < 100; i++) {
      data_filename[6] = '0' + i/10;
      data_filename[7] = '0' + i%10;
      if (!SD.exists(data_filename)) {
        datalog = SD.open(data_filename, FILE_WRITE);
        datalog.close();
        SD_data_active = true;
        printout("Logging data to: " + String(data_filename),true);
        break;
      }
    }
    if (!SD_report_active) printout("ERROR: No available report file names, clear SD card to enable logging",true);
    if (!SD_data_active) printout("ERROR: No available data file names, clear SD card to enable logging",true);
  }
}

void printout(String to_print, bool endline) {
  printout(to_print,endline,false);
}

void printout(String to_print, bool endline, bool data) {    //prints to both moniter(XBEE) and radio, endline is for print (false) or println (true), data tells what file to write to
  Serial2.print("$M$");
  Serial2.print(to_print);
  Serial.print(to_print);
  if(endline) {
    Serial2.println();
    Serial.println();
  }
  if(!(data) && SD_report_active) {
    report_data += to_print;
    extra_data = true;
    }
}

void printdata() {
  sensors.requestTemperatures();
  String data = gps_lock + String(sensors.getTempCByIndex(0)) + "," + String(ina219.getBusVoltage_V()) + "," + String(ina219.getShuntVoltage_mV()/1000.0) + "," 
  + String(ina219.getCurrent_mA()) + "," + String(ina219.getPower_mW()) + "," + String(geiger1.getTotalCount()) + "," + String(geiger1.getCycleCount());
  printout(data,true,true);
  if(extra_data) data += report_data;
  if(SD_data_active) {
    datalog = SD.open(data_filename, FILE_WRITE);
    datalog.println(data);
    datalog.close();
    extra_data = false;
    report_data = "";
  }
}

void radioAndGpsSetup() {
  xBee.init('A');
  gps.init();
}

void sensorsSetup() {
  sensors.begin();
  servo.attach(smartPin);
  servo.write(0);
  printout("SMART initialized",true);
  int32_t currentFrequency;
  ina219.begin();
  geiger1.init();
}

void checkStart() {
  if(analogRead(rbfPin) < 1000)  {
    printout("RBF wire pulled, flight started at ",false);
    flight_begun = true;
    millis_start_time = millis();
    printout(String(millis_start_time),true);   
  }
}

void checkDeploy()  {
  if(((millis()- millis_start_time) > (cutTime*60000)))  {
    servo.write(180);
    smart_release_attempt = true;
    printout("SMART triggered at " + String(millis()),true);
  }
}

void attemptGPS(bool override_gps) {
  gps_good = false;
  float gps_alt_current = 0.0;
  if(gps_alt[2] == 0.0) {
    override_gps = true;
    gps.update();
    gps_alt_current = gps.getAlt_meters();
  }
  if(!(override_gps)) {
    gps.update();
    gps_alt_current = gps.getAlt_meters();
    for(byte i=0; i<3; i++) {
      if(!((abs(gps_alt_current - gps_alt[i])) < (gpsTolerance * 1000 * (millis() - (millis_start_time + gps_time[i]))))) {
        override_gps = true;
      }
    }
    override_gps = !override_gps;
  }
  if(override_gps) {
    if(!(gps_alt[2] == 0)) gps_good = true;
    gps_alt[2] = gps_alt[1];
    gps_alt[1] = gps_alt[0];
    gps_alt[0] = gps_alt_current;
    gps_time[2] = gps_time[1];
    gps_time[1] = gps_time[0];
    gps_time[0] = millis();
  }
  gps.update();
  gps_lock = String(gps.getMonth()) + "/" + String(gps.getDay()) + "/" + String(gps.getYear()) + ","
                  + String(gps.getHour()) + ":" + String(gps.getMinute()) + ":" + String(gps.getSecond()) + ","
                  + String(gps.getLat(), 4) + "," + String(gps.getLon(), 4) + "," + String(gps_alt_current, 1) + ","
                  + String(gps.getSats()) + ",";
  if(gps.getFixAge() > cycle_time * 2) gps_lock += "No Fix,";
  else gps_lock += "Fix,";
}

void wait() {
  while(millis()-last_millis_overflow < cycle_time){
    gps.update();
  }
  last_millis_overflow = millis();
}

void recieveCommands()  {
  String command;
  if(Serial2.available() > 0) command = Serial2.readString();
  else {
    command = xBee.receive();
  }
  if ((!(command == ""))&&(!(command.substring(0,3).equals("$M$"))))  commandRegister(command);                //if a command is received, reads command and executes instructions
}

void resetGPS() {                                                 //experimental, copied from GPS example in MnSGC Training Catalogue
  for(int i=0;i<100;i++)  {
    gps.update();
    if(millis()%1000 == 0) {
    gps.update();
    String tempdata = String(gps.getMonth()) + "/" + String(gps.getDay()) + "/" + String(gps.getYear()) + ","
                  + String(gps.getHour()-5) + ":" + String(gps.getMinute()) + ":" + String(gps.getSecond()) + ","
                  + String(gps.getLat(), 4) + "," + String(gps.getLon(), 4) + "," + String(gps.getAlt_meters(), 1) + ","
                  + String(gps.getSats()) + ",";
    if(gps.getFixAge() > 2000)
      tempdata += "No Fix,";
    else
      tempdata += "Fix,";
    }
  }
}

void commandRegister(String command)  {
  printout("Command received: \"" + command + "\"",true);
  if(command.equals("QUERY") || command.equals("PING"))  {        //Radios back relevant status information, has no actual action
    if(flight_begun)  {
      printout("Flight time: " + String((millis()-millis_start_time)/60000.0) + " minutes, ",false);
      printout(String(((cutTime*60000)-(millis()- millis_start_time))/60000.0)+" Minutes till deployment, ",false);
    }
    else  printout("Flight not started, ",false);
    if(smart_release_attempt) printout("Deployed, ",false);
    else printout("Not deployed, ",false);
    printout("Cycle time is " + String(cycle_time),false);
    printout("",true);
  }
  else if(command.equals("TEMP"))  {                              //Takes and radios back temperature
    sensors.requestTemperatures();
    printout("Current temperature is " + String(sensors.getTempCByIndex(0)) + " degrees celsius",true);
  }
  else if(command.equals("GPS"))  {                               //Radios back gps infomation, has no actual action
    printout("Last lock at " + String((gps_time[0])/60000.0) + " minutes into flight (ERROR IF ZERO), most recent data:",true);
    printout(gps_lock,true);
  }
  else if(command.equals("GEIGER"))  {                            //Takes and radios back geiger counter hits, may mess up cycle count
    printout(String(geiger1.getCycleCount()) + " hits recieved since last cycle, " + String(geiger1.getTotalCount()) + " total hits",true);
  }
  else if(command.equals("VOLT"))  {                              //Takes and radios back voltage from panels
    printout("Current voltage is " + String(ina219.getBusVoltage_V() - (ina219.getShuntVoltage_mV()/1000.0)) + " volts",true);
  }
  else if(command.equals("DEPLOY"))  {                            //Attempts to deploy panels
    servo.write(180);
    smart_release_attempt = true;
    printout("SMART triggered at " + String(millis()) + " via command",true);
  }
  else if(command.equals("START"))  {                             //Activates measurement and sets start time if not already started
    printout("Flight started via command at ",false);
    if(!(flight_begun)) {
      flight_begun = true;
      millis_start_time = millis();
    }
    printout(String(millis()),true);   
  }
  else if(command.equals("ACGPS"))  {                             //Overrides GPS filter
    attemptGPS(true);
  }
  else if(command.equals("RSGPS"))  {                             //Spams GPS to establish link
    resetGPS();
    printout("GPS reset attempt completed",true);
  }
  else if((command.substring(0,4)).equals("SETT"))  {             //Changes the launch-till-deploy timer to inputted value, ie 'SETT35' would set timer to 35 minutes from now
    command.remove(0,4);
    cutTime = command.toFloat() + ((millis()-millis_start_time)/60000.0);
  }
  else if((command.substring(0,4)).equals("SETC"))  {             //Changes the time between logging and radio cycles to inputted value, ie 'SETC60' would set each cycle to a minute
    command.remove(0,4);
    cycle_time = command.toFloat() * 1000;
  }
  else if((command.substring(0,1)).equals("+"))  {                //Adds to launch-till-deploy timer by inputted value, ie '+7' would add 7 minutes to timer
    command.remove(0,1);
    cutTime += command.toFloat();
  }
  else if((command.substring(0,1)).equals("-"))  {                //Subtracts to launch-till-deploy timer by inputted value, ie '+7' would subtract 7 minutes from timer
    command.remove(0,1);
    cutTime -= command.toFloat();
  }
  else if((command.substring(0,1)).equals("P"))  {                //Forwards message to PPOD, ie 'P+15' would relay '+15' to any connectted XBee with ID 'POD' or 'PPOD'
    command.remove(0,1);
    Serial.println("POD?" + command + "!");
    Serial.println("PPOD?" + command + "!");
  }
  else if(!(command.equals("")))  {
    printout("ERROR: Command not recognized",true);
  }
}
