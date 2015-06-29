//TrackPack
//Portable security system
//Created by Tris Mendoza, Emily Dao, Allen Chan, and Ellen Liu


#include <SPI.h>
#include "Adafruit_BLE_UART.h"

//Setup Bluetooth REQ, RED, RST pins
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

//setup LED/buzzer pins
#define BUZZER 8
#define BLUE 7 //blue LED
#define GREEN 6  //green LED
#define RED 5  //red LED
#define BUTTON 4

//accelerometer setup
int scale = 3;
boolean micro_is_5V = true; //using Arduino Uno
int iniX, iniY, iniZ;
boolean alarmon = false;
int lastStatus = ACI_EVT_DISCONNECTED;
Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
boolean found = true;
boolean command = false;
void setup(){
  //Initialize serial communication @ 115200 baud
  Serial.begin(115200);
  Serial.println(F("Trackpack - Bluetooth Low Energy"));
  uart.begin();
  //Set up X, Y, Z analog read for the accelerometer
  iniX = analogRead(A0);
  iniY = analogRead(A1);
  iniZ = analogRead(A2);
  
  //Set up buzzer/RGB LED pins and set to low initially
  pinMode(BUTTON, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  digitalWrite(BUZZER, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
}

void loop(){
  
  uart.pollACI();
  
  int status = uart.getState();
  
  if(status != lastStatus){
    if(status == ACI_EVT_DEVICE_STARTED){
      //turn on blue LED if bluetooth has started advertising
      Serial.println(F(" * Advertising Started"));
      digitalWrite(RED, LOW);
      digitalWrite(BLUE, HIGH);
      digitalWrite(GREEN, LOW);
    }
    else if (status == ACI_EVT_CONNECTED){
      //once connected to the app, turn on the green LED
      Serial.println(F(" * Connected!"));
      uart.println("TrackPack Connected!");
      digitalWrite(BLUE, LOW);
      digitalWrite(GREEN, HIGH);
      digitalWrite(RED, LOW);
    } 
    else if (status == ACI_EVT_DISCONNECTED){
      Serial.println(F(" *Disconnected or advertising timed out."));
      digitalWrite(BLUE, LOW);
    }
    
    lastStatus = status;
  }
  
  if(status == ACI_EVT_CONNECTED){
    //if the bluetooth is connected, check for accelerometer values
    int rawX = analogRead(A0);
    int rawY = analogRead(A1);
    int rawZ = analogRead(A2);
    
   
    //if the change in the accelerometer values is too much, this means the device has been moved
    //if it's moved, turn the alarm on
    if(rawX>(iniX+50) || rawY > (iniY+50) || rawZ > (iniZ + 50) || rawX<(iniX-50) || rawY<(iniY-50) || rawZ < (iniZ-50)){
      alarmon = true;
    }
    
    //check x values
    Serial.print("RawX: "); Serial.println(rawX);
    Serial.print("IntX: "); Serial.println(iniX);
    Serial.print("RawY: "); Serial.println(rawY);
    Serial.print("IntY: "); Serial.println(iniY);
    Serial.print("RawZ: "); Serial.println(rawZ);
    Serial.print("IntZ: "); Serial.println(iniZ);

    if(alarmon){
      //if alarm is on, turn on the red LED and the buzzer
      alarmLED();
      buzz();
      //print "ALARM" on the app to alert the user
      uart.println("ALARM!");
      //if the user types "OFF", this turns off the alarm
      if(uart.find("OFF")){
        alarmOFF();
        uart.println("Alarm disabled");
        alarmon = false;
        iniX = rawX;
        iniY = rawY;
        iniZ = rawZ;
      } 
    }
    
    if(uart.find("FIND")){
        uart.println("Alarm enabled"); 
        findLED();
        //buzz(); 
        while(!digitalRead(BUTTON)){
          buzz();
          delay(200);
          noTone(BUZZER);
          delay(200);
        }
        alarmOFF();
        uart.println("Alarm disabled");
      }
   
    }
    }
  

//control buzzer
void buzz(){
  tone(BUZZER, 400, 250);
  delay(250);
  noTone(BUZZER);
}

//alarm LED turns on the red LED
void alarmLED(){
  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, LOW);
  digitalWrite(GREEN, LOW);
}

//returns to alarm state, so blue LED is on
void alarmOFF(){
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(GREEN, HIGH);
}

//find LED, so make the LED blink white
void findLED(){
    digitalWrite(RED, HIGH);
    digitalWrite(BLUE, HIGH);
    digitalWrite(GREEN, HIGH);

}

