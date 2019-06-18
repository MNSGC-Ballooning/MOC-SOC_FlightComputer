#include "src\Smart.h"
#include <SoftwareSerial.h>
#include <Relay_XBee.h>

#define smartPin 10
#define light 9 
#define onlightn 7 //The light that turns on when it is powered up and remains on
#define onlightp 6 // See above

bool released = false;
String ID = "SOC";
String command;
float cutTime = 70; //Time in minutes
float cutAlt = 18288; //Alt in meters
float currentAlt = 0;

SoftwareSerial ss = SoftwareSerial(2,3);
XBee xBee = XBee(&ss, ID);

Smart smart = Smart(smartPin);

void setup() {
  smart.initialize();
  //gps.initialize();
  //pinMode(light, OUTPUT);
  pinMode(onlightn, OUTPUT);
  pinMode(onlightp, OUTPUT);
  ss.begin(9600);
  //Serial.println("Testing");
  digitalWrite(onlightn, LOW);
  digitalWrite(onlightp, HIGH);
}

void loop() {
  //gps.update();
  if (released == false) {
    digitalWrite(onlightn,HIGH);
    digitalWrite(onlightp,LOW);
    delay(1500);
    digitalWrite(onlightn,LOW);
    digitalWrite(onlightp,HIGH);
    delay(1500);
  }

  if (!released && ((millis()/60000.0 > cutTime-.2)) || (currentAlt < cutAlt)) {
    digitalWrite(light, HIGH);}
  if (!released && ((millis()/60000.0 > cutTime)) || (currentAlt >= cutAlt)) {
    //ss.println("PPOD: Releasing");
    int counter = 0;
    while (counter < 16) {
      digitalWrite(onlightp, HIGH);
      digitalWrite(onlightn, LOW);
      delay(200);

      counter += 1;
    }
  
    smart.release();
    
    released = true;

    //ss.println("PPOD released");
  }

  if (released == true) {
    digitalWrite(onlightn, HIGH);
    digitalWrite(onlightp, LOW);
    delay(500);
    digitalWrite(onlightn, LOW);
    digitalWrite(onlightp, HIGH);
    delay(500);
  }

  if (ss.available() > 0) {
    command = ss.readString();
    if (command == "timeleft") {
      ss.print(",,,,,,,,,,,,PPOD ," + String(cutTime - millis()/60000));
    }
    else if (command.startsWith("+")) {
      command.remove(0,1);
      float addtime = command.toFloat(); 
      cutTime = cutTime + addtime;
      ss.print(",,,,,,,,,,,,PPOD: " + String(addtime) + " Minutes added." + " New Time Left: ," + String(cutTime - millis()/60000));
    }
    else if (command.startsWith("-")) {
      command.remove(0,1);
      float subtime = command.toFloat();
      cutTime = cutTime - subtime;
      ss.print(",,,,,,,,,,,,PPOD " + String(subtime) + " Minutes removed." + " New Time Left: ," + String(cutTime - millis()/60000));
    }
    else if (command.startsWith("add")) {
      command.remove(0,3);
      float addalt = command.toFloat();
      cutAlt = cutAlt + addalt;
      ss.print(",,,,,,,,,,,,PPOD " + String(addalt) + " Meters added. New distance left: ,," + String(cutAlt - currentAlt));
    }
    else if (command.startsWith("sub")) {
      command.remove(0,3);
      float subalt = command.toFloat();
      cutAlt = cutAlt - subalt;
      ss.print(",,,,,,,,,,,,PPOD " + String(subalt) + " Meters lost. New distance left: ,," + String(cutAlt - currentAlt));
    }
    else if (command == "distleft") {
      ss.print(",,,,,,,,,,,,PPOD ,," + String(cutAlt-currentAlt));
    }
    else if (command.startsWith("ALT")) {
      command.remove(0,3);
      currentAlt = command.toFloat(); 
    }
    else if (command == "cut") {
      if (released == true) {
        ss.print(",,,,,,,,,,,,PPOD: Already Released");
      }
      if (released == false) {
        smart.release();
        released = true;
        ss.print(",,,,,,,,,,,,PPOD: Released");
      }
    }
    else if (command == "PPOD") {
      if (released == false) {
      ss.print(",PPOD:, " + String(cutTime - millis()/60000) + "," + String(cutAlt - currentAlt));
      }
      else if (released == true) {
        ss.print(",PPOD:, Released");
      }
    }
    else if (command == "humidity" || command == "pressure" || command == "1u-temp" || command == "1U Data") {
      return;
    }
  }
}

