//MOC-SOC Flight Computer
//Written by Paul Wehling with assistance from Lilia Bouayed, Based on code originally written by Jordan Bartlett and Emma Krieg.
//For use with MOC-SOC 3U Cubesat 1.2, built by Paul Wehling and Donald Rowell, based on original by Jordan Bartlett and Emma Krieg.
//Current code built to run alongside ParasitePCB code on PPOD Deployment System.
//Last updated August 11, 2020

  #include <Servo.h>
  #include <SD.h>
  #include <DallasTemperature.h>
  #include <OneWire.h>
  #include <Adafruit_INA219.h>
  #include <GeigerCounter.h>
  #include <RelayXBee.h>
  #include <UbloxGPS.h>       //MNSGC "FlightGPS" library

  #define chipSelect 4        //primary pin for SD Logging
  #define tempBus 10          //data pin for temperature probe
  #define smartPin 6          //digital pin for SMART release system
  #define rbfPin A0           //pin through which remove-before-flight pin connects to 5V rail
  #define radioConnPin A1     //pin that monitiers radio's connection status
  #define timeTillDeploy 75   //flight time in minutes after which to deploy panels
  #define northFence 44.63    //north gps fence for deployment, -200 to deactivate
  #define eastFence -92.52    //east gps fence for deployment, -200 to deactivate
  #define southFence 43.50    //south gps fence for deployment, -200 to deactivate
  #define westFence -95.75    //west gps fence for deployment, -200 to deactivate
  #define altFence 75000      //altitude (in ft) for deployment, -200 to deactivate
  #define xbeeSerial Serial   //communication channel for shield XBee
  #define cycle_time_sec 5    //time in seconds for a complete logging cycle
  #define ubloxSerial Serial1 //communication channel for UBLOX GPS
  #define gpsTolerance 25     //ft/s the gps is allowed to drift from last contact
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
  bool ppod_deploy_ping = false;                                  //tracks if PPOD has deployed, allows MOC-SOC deployment after recieved ping. Ignored by 'DEPLOY' command
  bool radio_tick = false;                                        //tracks if radio has made a connection yet during cycle
  bool smart_telemetry = true;                                    //tracks if advanced telemetry is enabled, flips if SD card fails during bootup
  unsigned long last_report_relayed = 0;                          //tracks last sent report file line
  unsigned long last_data_relayed = 0;                            //tracks last sent data file line
  float n_fence = northFence;                                     //gps fence value for solar arm deployment, if -200 fence deactivated
  float e_fence = eastFence;                                      //gps fence value for solar arm deployment, if -200 fence deactivated
  float s_fence = southFence;                                     //gps fence value for solar arm deployment, if -200 fence deactivated
  float w_fence = westFence;                                      //gps fence value for solar arm deployment, if -200 fence deactivated
  float a_fence = altFence;                                       //altitude fence value for solar arm deployment, if -200 fence deactived
  bool dply_geo_fence = true;                                     //toggles gps (NESW) fence usage, if true gps fences can trigger deployment
  bool dply_alt_fence = true;                                     //toggles altitude fence usgae, if true altitude fence can trigger deployment
  bool dply_timer = true;                                         //toggles timer usage, if true flight timer can trigger deployment
  bool dply_descent = true;                                       //toggles automatic deploy after detecting burst, if true any descent triggers deployment
  bool force_ppod_deploy = true;                                  //toggles automatic backup PPOD deploy, if true PPOD will recive deploy command when MOC-SOC attempts deployment
  unsigned long tracked_millis = 0;                                //holds last millis() value betweeen cycles for data clarity check
  

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
  recieveCommands();                                              //checks XBee and freewave for commands from ground/comms/etc
  attemptGPS(false);                                              //tries to establish a gps lock and filters for good and bad hits
  printdata();                                                    //prints the data to radio, serial, SD, and XBEE
  wait();                                                         //holds until cycle time has elapsed
}

void sdSetup() {                 //starts SD Card logging and creates logging files
  printout("Initializing SD card",true);
  if(!SD.begin(chipSelect)) printout("ERROR: Card failed, or not present",true);
  else {
    printout("Card initialized.\nCreating Files",true);
    for (byte i = 0; i < 100; i++) {
      report_filename[6] = '0' + i/10;
      report_filename[7] = '0' + i%10;
      if (!SD.exists(report_filename)) {
        datalog = SD.open(report_filename, FILE_WRITE);
        datalog.println(""); //loads blank line into report so radioConnect doesn't miss it
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
        datalog.println(""); //loads blank line into data so radioConnect doesn't miss it
        datalog.close();
        SD_data_active = true;
        printout("Logging data to: " + String(data_filename),true);
        break;
      }
    }
    if (!SD_report_active) printout("ERROR: No available report file names, clear SD card to enable logging",true);
    if (!SD_data_active) printout("ERROR: No available data file names, clear SD card to enable logging",true);
  }
  if (SD_report_active && SD_data_active) smart_telemetry = true;
  else printout("WARNING: Smart telemetry disabled", true);
}

void printout(String to_print, bool endline) {
  printout(to_print,endline,false);
}

void printout(String to_print, bool endline, bool data) {    //prints to both moniter(XBEE) and radio, endline is for print (false) or println (true), data tells what file to write to
  Serial.print("$M$" + to_print);
  if (endline) Serial.println();
  if(!(data) && SD_report_active) {
    report_data += to_print;
    if(endline) report_data += '\n';
    extra_data = true;
    }
  if(!smart_telemetry) {
    Serial2.print("$M$" + to_print);
    if(endline) Serial2.println();
  }
}

void printdata() {
  sensors.requestTemperatures();
  String data = gps_lock + String(sensors.getTempCByIndex(0)) + "," + String(ina219.getBusVoltage_V()) + "," + String(ina219.getShuntVoltage_mV()/1000.0) + "," 
  + String(ina219.getCurrent_mA()) + "," + String(ina219.getPower_mW()) + "," + String(geiger1.getTotalCount()) + "," + String(geiger1.getCycleCount()) + ","
  + String(ppod_deploy_ping) + "," + String(tracked_millis) + ",";
  tracked_millis = millis();
  data += String(tracked_millis) + "," + String(tracked_millis);
  printout(data,true,true);
  if(SD_data_active) {
    datalog = SD.open(data_filename, FILE_WRITE);
    datalog.println(data);
    datalog.close();
  }
  if(SD_report_active){
    datalog = SD.open(report_filename, FILE_WRITE);
    datalog.print(report_data);
    datalog.close();
    extra_data = false;
    report_data = "";
  }
}

void radioAndGpsSetup() {
  xBee.init('C');
  xBee.enterATmode();
  xBee.atCommand("ATDL0");
  xBee.atCommand("ATMY1");
  xBee.exitATmode();
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
  if(dply_timer && ((millis()- millis_start_time) > (cutTime*60000)))  attemptDeploy();
  if((dply_geo_fence || dply_alt_fence) && gps_good)  {
    String temp_coords = gps_lock;
    float temp_calc = 0;
    for(int i=0; i<2; i++)  {
      temp_coords.remove(0,temp_coords.indexOf(','));
      temp_calc = temp_coords.substring(0,temp_coords.indexOf(',')).toFloat();
      if(dply_geo_fence && (((s_fence != -200) && (temp_calc < s_fence)) || ((n_fence != -200) && (temp_calc > n_fence)))) attemptDeploy();
      temp_coords.remove(0,temp_coords.indexOf(','));
      temp_calc = temp_coords.substring(0,temp_coords.indexOf(',')).toFloat();
      if(dply_geo_fence && (((w_fence != -200) && (temp_calc < w_fence)) || ((e_fence != -200) && (temp_calc > e_fence)))) attemptDeploy();
      temp_coords.remove(0,temp_coords.indexOf(','));
      temp_calc = temp_coords.substring(0,temp_coords.indexOf(',')).toFloat();
      if(dply_alt_fence && (temp_calc > a_fence)) attemptDeploy();
    }
  }
  if(dply_descent && gps_good && (gps_alt[0] < gps_alt[1]) && (gps_alt[1] < gps_alt[2]))  attemptDeploy();
}

void attemptDeploy()  {
  servo.write(180);
  smart_release_attempt = true;
  printout("SMART triggered ar " + String(millis()),true);
  if(force_ppod_deploy) Serial.println("PPOD?CUT!");
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
      if(!((abs(gps_alt_current - gps_alt[i])) < (gpsTolerance * 1000 * (millis() - gps_time[i])))) {
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
  if(gps.getFixAge() > cycle_time * 2) {
    gps_lock += "No Fix,";
    gps_good = false;
  }
  else gps_lock += "Fix,";

  if(gps_good) gps_lock += "GPSCLR,";
  else gps_lock += "GPSBAD,";
}

void wait() {
  while(millis()-last_millis_overflow < cycle_time){
    gps.update();
    recieveCommands();
    if(!radio_tick && smart_telemetry) radioConnect();
    delay(10);
  }
  last_millis_overflow = millis();
  radio_tick = false;
}

void recieveCommands()  {
  String command;
  if(Serial2.available() > 0) command = Serial2.readString();
  if ((!(command == ""))&&(!(command.substring(0,3).equals("$M$"))))  commandRegister(command);                //if a command is received, reads command and executes instructions
  
  if(Serial.available() > 0) command = xBee.receive();
  {
    if (command.substring(0,3).equals("$P$"))  {
      printout("Forwarded from XBEE: \"",false);
      short cmd_length = command.length();
      printout(command.substring(4,cmd_length),false);
      printout("\"",true);
    }
    else if (command.equals("R1"))  {
      ppod_deploy_ping = true;
    }
    else if (command.equals("R0"));
    else if (!command.equals("")) commandRegister(command);
  }
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
  else if(command.equals("PREPDEPLOY"))  {                        //Removes need for MOC-SOC to recieve go-ahead from PPOD to deploy
    if (!ppod_deploy_ping) {
      ppod_deploy_ping = true;
      printout("Ready to deploy without PPOD authorization",true);
    }
  }
  else if(command.equals("UNPREPDEPLOY"))  {                      //Undoes PREPDEPLOY command.  Will not stop deployment if PPOD is broacasting a deployed signal
    if (ppod_deploy_ping) {
      ppod_deploy_ping = false;
      printout("Ready to deploy with PPOD authorization",true);
    }
  }
  else if(command.equals("RESENDALL"))  {                         //Rapidly sends all data and reports taken so far in flight
    last_data_relayed = 0;
    last_report_relayed = 0;
    radioConnect();
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
  else if(command.equals("TOGGLERADIOMODE"))  {                   //Switches smart telemetry on or off
    smart_telemetry = !smart_telemetry;
    if(smart_telemetry) printout("WARNING: Smart telemetry enabled. Data and report relay may be lost if SD card error",true);
    else printout("WARNING: Smart telemetry disabled. Data and report relay may be lost if connection error",true);
  }
  else if ((command.substring(0,10)).equals("RESENDDATA"))  {     //Resends inputted value of last data, ie 'RESENDDATA30' would send the 30 most recent data lines
    command.remove(0,10);
    last_data_relayed -= command.toFloat();
    radioConnect();
  }
  else if ((command.substring(0,12)).equals("RESENDREPORT"))  {   //Resends inputted value of last reports, ie 'RESENDREPORT30' would send the 30 most recent data lines
    command.remove(0,12);
    last_data_relayed -= command.toFloat();
    radioConnect();
  }
  else if((command.substring(0,5)).equals("FENCE"))  {            //Allows changing of gps and altitude fences, i.e. 'FENCEN-200' would set the north fence to -200, or off
    switch(command.charAt(5)) {
      case('N'): n_fence = (command.substring(6)).toFloat();
      case('E'): e_fence = (command.substring(6)).toFloat();
      case('S'): s_fence = (command.substring(6)).toFloat();
      case('W'): w_fence = (command.substring(6)).toFloat();
      case('A'): a_fence = (command.substring(6)).toFloat();
      case(26):;
      default: printout("WARNING: Fence command not recognized",true);
    }
    printout("Fences set to " + String(n_fence) + "N, " + String(s_fence) + "S, " + String(e_fence) 
      + "E, " + String(w_fence) + "W, " + String(a_fence) + "alt(ft)",true);
  }
  else if((command.substring(0,9)).equals("SETDEPLOY"))  {        //Toggles deployment booleans, G for GPS, A for altitude, T for timer, D for descent, P for forced PPOD deploy
    switch(command.charAt(9)) {
      case('G'): dply_geo_fence = !dply_geo_fence;
      case('A'): dply_alt_fence = !dply_alt_fence;
      case('T'): dply_timer = !dply_timer;
      case('D'): dply_descent = !dply_descent;
      case('P'): force_ppod_deploy = !force_ppod_deploy;
      case(26):;
      default: printout("WARNING: Deployment command not recognized",true);
    }
    printout("Deployment bools set to " + String(dply_geo_fence) + "G " + String(dply_alt_fence) 
      + "A " + String(dply_timer) + "T " + String(dply_descent) + "D " + String(force_ppod_deploy) 
      + "P",true);
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
  else if((command.substring(0,1)).equals("P"))  {                //Forwards message to PPOD, ie 'P+15' would relay '+15' to any connected XBee with ID 'PPOD'
    command.remove(0,1);
    Serial.println("PPOD?" + command + "!");
  }
  else if(!(command.equals("")))  {
    printout("ERROR: Command not recognized",true);
  }
}

void radioConnect() {
  bool data_done = false;
  bool report_done = false;
  bool good_conn = false;
  char next_ch;
  if(analogRead(radioConnPin) > 500) {
    good_conn = true;
    datalog = SD.open(data_filename, FILE_READ);
    next_ch=datalog.read();
    for (int i=0; i<=last_data_relayed; i++) {
      do next_ch=datalog.read(); while(next_ch != '\n');
    }
    good_conn = (analogRead(radioConnPin) > 500);
    if(!(datalog.peek()+1)) data_done = true;
    while(good_conn && !data_done) {
      Serial2.print("$M$");
      do {
        next_ch=datalog.read();
        Serial2.print(next_ch);
      } while(next_ch != '\n');
      good_conn = (analogRead(radioConnPin) > 500);
      if(good_conn) last_data_relayed++;
      if(!(datalog.peek()+1)) data_done = true;
    }
    datalog.close();
    
    datalog = SD.open(report_filename, FILE_READ);
    next_ch=datalog.read();
    for (int i=0; i<=last_report_relayed; i++) {
      do next_ch=datalog.read(); while(next_ch != '\n');
    }
    good_conn = (analogRead(radioConnPin) > 500);
    if(!(datalog.peek()+1)) report_done = true;
    while(good_conn && !report_done) {
      Serial2.print("$M$");
      do {
        next_ch=datalog.read();
        Serial2.print(next_ch);
      } while(next_ch != '\n');
      good_conn = (analogRead(radioConnPin) > 500);
      if(good_conn) last_report_relayed++;
      if(!(datalog.peek()+1)) report_done = true;
    }
    datalog.close();
    printout(("Connection made at " + String(millis()) + ", data relayed to line " + 
    String(last_data_relayed) + ", reports to line " + String(last_report_relayed)),true);
    if(report_done && data_done) radio_tick = true;
  }
}
