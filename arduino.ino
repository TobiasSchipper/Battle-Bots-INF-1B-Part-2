/*
 * ==========================================================
 *  Project: Battlebots
 *  Board  : Arduino Nano
 *  Author : Robbin & Tobias Schipper
 *  Version: 1.0
 *  
 *
 * ==========================================================
 *  PINOUT ARDUINO NANO
 *  ----------------------------------------------------------
 *  | Pin | Functie      | Opmerking                         |
 *  |-----|-------------|----------------------------------|
 *  | D0  | RX          | Seriële communicatie (in)       |
 *  | D1  | TX          | Seriële communicatie (uit)      |
 *  | D2  | Digitaal I/O| Data pin voor neopixels         |
 *  | D3  | Digitaal I/O| Input IfrSensor ECHO            |
 *  | D4  | Digitaal I/O| MH-Sensor voor motor links      |
 *  | D5  | Digitaal I/O| PWM motor rechts -              |
 *  | D6  | Digitaal I/O| PWM motor rechts +              |
 *  | D7  | Digitaal I/O| vrij                            |
 *  | D8  | Digitaal I/O| MH-Sensor rechts                |
 *  | D9  | Digitaal I/O| Output IfrSensor TRIG           |
 *  | D10 | Digitaal I/O| PWM motor links -               |
 *  | D11 | Digitaal I/O| PWM motor links +               |
 *  | D12 | Digitaal I/O| vrij                            |
 *  | D13 | Digitaal I/O| Input gripper                   |
 *  | A0  | Analoog In  | LijnSensor 1                    |
 *  | A1  | Analoog In  | LijnSensor 2                    |
 *  | A2  | Analoog In  | LijnSensor 3                    |
 *  | A3  | Analoog In  | LijnSensor 4                    |
 *  | A4  | Analoog In  | LijnSensor 5                    |
 *  | A5  | Analoog In  | LijnSensor 6                    |
 *  | A6  | Analoog In  | LijnSensor 7                    |
 *  | A7  | Analoog In  | LijnSensor 8                    |
 *  -------------------------------------------------------
 */

 //-------------LIBRARIES-----------------------------------------
#include <Adafruit_NeoPixel.h>

//-------------NEO PIXEL SETUP-----------------------------------
#define PIN 2       // Data pin connected to DIN
#define NUMPIXELS 4  // Change to match your LED count
#define INTERVAL 125  // Speed of the rainbow effect
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//-------------PIN SETUP------------------------------------------
#define MOTORA1 11  // Richting & PWM
#define MOTORA2 10  // Richting & PWM
#define SENSOR_R1 4 // MH-sensor links

//-------------Rechts motor
#define MOTORB1 5   // Richting & PWM
#define MOTORB2 6   // Richting & PWM
#define SENSOR_R2 8 // MH-sensor rechts

//-------------IFR Sensor
#define TRIGPIN 9
#define ECHOPIN 3
float duration, distance;

//-------------Gripper
#define GRIPPER 13

  //-------------SETUP
void setup() {
    Serial.begin(9600);

    //motor setup
    pinMode(MOTORA1, OUTPUT);
    pinMode(MOTORA2, OUTPUT);
    pinMode(MOTORB1, OUTPUT);
    pinMode(MOTORB2, OUTPUT);

    pinMode(SENSOR_R1, INPUT_PULLUP);
    pinMode(SENSOR_R2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SENSOR_R1), countPulsesA, RISING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_R2), countPulsesB, RISING);

    // knop setup
    pinMode(buttonPin2, INPUT);
    //IFR Setup
    pinMode(TRIGPIN, OUTPUT);  
    pinMode(ECHOPIN, INPUT); 
    //gripper setup
    pinMode(GRIPPER, OUTPUT); 

    //Pixel setup
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}

//-------------LOOP
void loop() {
    ifrSensor();
    ifrInformation();
    if(distance < 15)
    {
        stopMotor();
    }
    else
    {
        forward();
    }
    // Handle periodic tasks without delay()
    draaiendeSirene(); // Call the siren function without delay
}

//-------------GRIPPER FUNCTIE
void generatePulse(int angle){
    int pulseWidth = map(angle, 0, 180, 544, 2400);
    digitalWrite(GRIPPER, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(GRIPPER, LOW);
}

//-------------GRIPPER OPEN
void gripperOpen(){
    generatePulse(110);
}

//-------------GRIPPER SLUITEN
void gripperClosed(){
    generatePulse(48);
}

//-------------ON/OFF IFRSENSOR
void ifrSensor(){
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);
}

//-------------HAAL AFSTAND OP VAN IFRSENSOR
void ifrInformation(){
    duration = pulseIn(ECHOPIN, HIGH);
    distance = (duration*.0343)/2;
    Serial.print("Distance: ");
    Serial.println(distance);
    delayMicroseconds(100);
}