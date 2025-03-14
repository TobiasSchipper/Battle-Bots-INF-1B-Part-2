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
*  ---------------------------------------------------------
*  | Pin | Functie      | Opmerking                        |
*  |-----|-------------|----------------------------------|
*  | D0  | RX          | Seriële communicatie (in)       |
*  | D1  | TX          | Seriële communicatie (uit)      |
*  | D2  | Digitaal I/O| Data pin voor neopixels         |
*  | D3  | Digitaal I/O| Input IfrSensor ECHO            |
*  | D4  | Digitaal I/O| MH-Sensor voor motor links      |
*  | D5  | Digitaal I/O| PWM motor rechts -              |
*  | D6  | Digitaal I/O| PWM motor rechts +              |
*  | D7  | Digitaal I/O| Button 2                        |
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
#define NUMPIXELS 4  // neopixel led count
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

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
float _DURATION, _DISTANCE;

//-------------Button Pin
#define buttonPin2 7

//-------------Gripper
#define GRIPPER 13

//-----------lijnsensor
#define _NUM_SENSORS 8
const int LineSensor[_NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

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

    // Zet alle sensors als input
    for (int i = 0; i < _NUM_SENSORS; i++) {
        pinMode(LineSensor[i], INPUT);
    }
}

//-------------LOOP
void loop() {
    ifrSensor();
    ifrInformation();
    if(_DISTANCE <= 15) {
        right45();
        forward(1000);
        left45();
        forward(1500);
        left45();
        forward(1000);
        right45();
    }
    else {
        forward(0);
    }
}

//--------------------BEWEGINGSFUNCTIES
//-------------DEFAULT FUNCTION
void motorControl(int motorA1, int motorA2, int motorB1, int motorB2) {
    analogWrite(MOTORA1, motorA1);
    analogWrite(MOTORA2, motorA2);
    analogWrite(MOTORB1, motorB1);
    analogWrite(MOTORB2, motorB2);
}

//-------------FORWARD
void forward(int interval) {
    regularLight();
    motorControl(0, 168, 0, 200);
    delay(interval);
}

//-------------BACKWARD
void backward() {
    brakeLight();
    motorControl(168, 0, 200, 0);
}

//-------------RIGHT 45 DEGREE
void right45() {  
    unsigned long startTime = millis();
    while (millis() - startTime < 237) { 
        motorControl(0, 255, 255, 0);
        blinkerRight();
    }
    stopMotorControl(); // Stop the motor after the turn
}

//-------------RIGHT 90 DEGREE
void right90() {
    unsigned long startTime = millis();
    while (millis() - startTime < 475) { 
        motorControl(0, 255, 255, 0);
        blinkerRight();
    }
    stopMotorControl(); // Stop the motor after the turn
}

//-------------LEFT 45 DEGREE
void left45() {
    blinkerLeft();
    unsigned long startTime = millis();
    motorControl(255, 0, 0, 255);
    while (millis() - startTime < 237) { // Adjust the duration as needed
    // Wait for the desired duration in milliseconds
    }
}

//-------------LEFT 90 DEGREE
void left90() {
    blinkerLeft();
    unsigned long startTime = millis();
    motorControl(255, 0, 0, 255);
    while (millis() - startTime < 450) { // Adjust the duration as needed
        // Wait for the desired duration in milliseconds
    }
}

//-------------STOP MOTOR CONTROL
void stopMotorControl() {
    regularLight();
    motorControl(0, 0, 0, 0);
}

//-------------GRIPPER FUNCTIE
void generatePulse(int angle) {
    int pulseWidth = map(angle, 0, 180, 544, 2400);
    digitalWrite(GRIPPER, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(GRIPPER, LOW);
}

//-------------GRIPPER OPEN
void gripperOpen() {
    generatePulse(110);
}

//-------------GRIPPER SLUITEN
void gripperClosed() {
    generatePulse(48);
}

//-------------ON/OFF IFRSENSOR
void ifrSensor() {
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);
}

//-------------HAAL AFSTAND OP VAN IFRSENSOR
void ifrInformation() {
    _DURATION = pulseIn(ECHOPIN, HIGH);
    _DISTANCE = (_DURATION*.0343)/2;
    Serial.print("Distance: ");
    Serial.println(_DISTANCE);
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 100) { // 100 millisecond delay
        lastMillis = millis();
    }
}

//-----------------LICHT FUNCTIES
//-------------DEFAULT INDICATOR PROGRAM
void blinkers(int boven, int onder, bool active) {
    static unsigned long previousMillis = 0;
    static bool ledState = false;  // Ensure ledState is retained across calls
    unsigned long currentMillis = millis();

    if (active == true) {  // Alleen knipperen als "active" true is
        if (currentMillis - previousMillis >= 500) {  
            previousMillis = currentMillis;
            ledState = !ledState;  // Toggle state
        }
    } else {
        ledState = false;
        previousMillis = 0;  // LED blijft uit als knipperen stopt
    }

    if (ledState) {
        strip.setPixelColor(boven, strip.Color(255, 69, 0));  // Orange ON
        strip.setPixelColor(onder, strip.Color(255, 69, 0));  // Orange ON
    } else {
        strip.setPixelColor(boven, strip.Color(100, 0, 0));  // Red
        strip.setPixelColor(onder, strip.Color(100, 100, 100)); // White
    }

    strip.show();  // Update the strip to reflect changes
}



//-------------LEFT BLINKER
void blinkerLeft() {
    strip.clear();
    blinkers(0, 3, true);
    strip.setPixelColor(1, strip.Color(150, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.show();
}

//-------------RIGHT BLINKER
void blinkerRight() {
    strip.clear();
    blinkers(1, 2, true);
    strip.setPixelColor(0, strip.Color(150, 0, 0));  // red
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
}

//-------------BRAKE LIGHT
void brakeLight() {
    strip.clear();
    strip.setPixelColor(0, strip.Color(255, 0, 0));  // red
    strip.setPixelColor(1, strip.Color(255, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
    }

//-------------DEFAULT LIGHT
void regularLight() {
    strip.clear();
    strip.setPixelColor(0, strip.Color(50, 0, 0));  // red
    strip.setPixelColor(1, strip.Color(50, 0, 0));  // red
    strip.setPixelColor(2, strip.Color(150, 150, 150));  // white
    strip.setPixelColor(3, strip.Color(150, 150, 150));  // white
    strip.show();
}