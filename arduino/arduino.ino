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
*  | A0  | Analoog In  | LijnSensor 1 Rechts             |
*  | A1  | Analoog In  | LijnSensor 2                    |
*  | A2  | Analoog In  | LijnSensor 3                    |
*  | A3  | Analoog In  | LijnSensor 4                    |
*  | A4  | Analoog In  | LijnSensor 5                    |
*  | A5  | Analoog In  | LijnSensor 6                    |
*  | A6  | Analoog In  | LijnSensor 7                    |
*  | A7  | Analoog In  | LijnSensor 8 Links              |
*  -------------------------------------------------------
*/

//-------------LIBRARIES-----------------------------------------
#include <Adafruit_NeoPixel.h>

//-------------NEO PIXEL SETUP-----------------------------------
#define PIN 2       // Data pin connected to DIN
#define NUMPIXELS 4  // neopixel led count
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

//-------------PIN SETUP------------------------------------------
//-------------Links motor
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

//-------------Button Setup
#define BUTTONPIN2 7
bool buttonState = 0;

//-------------Gripper
#define GRIPPER 13

//-----------lijnsensor
#define _NUM_SENSORS 8
const int LineSensor[_NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};
int sensorReadings[_NUM_SENSORS];
int _DEADZONELOW = 0;
int _DEADZONEHIGH = 0;
int _LASTDISTANCE = 0;

//-----------DEBUGGING
//#define SENSORVALUE

//-----------OVERIGE VARIABELEN
bool isCalibrated = false;

//-----------MOTOR SNELHEDEN
#define FULLSPEED 255
#define STEADY_SPEED 214
#define SLOW_SPEED 180
#define SLOWER_SPEED 60
#define SLOWEST_SPEED 10
#define SEARCH_SPEED 50

//-----------LINEFOLLOW LOGIC
int _lastDirection = 0;  // 0 = recht, -1 = links, 1 = rechts
const float rightTurnSpeedFactor = 0.9; // Verlaag rechter motor met 10%

//-------------SETUP
void setup() {
    Serial.begin(9600);

    //motor setup
    pinMode(MOTORA1, OUTPUT);
    pinMode(MOTORA2, OUTPUT);
    pinMode(MOTORB1, OUTPUT);
    pinMode(MOTORB2, OUTPUT);

    // knop setup
    pinMode(BUTTONPIN2, INPUT);
    //IFR Setup
    pinMode(TRIGPIN, OUTPUT);  
    pinMode(ECHOPIN, INPUT); 
    //gripper setup
    pinMode(GRIPPER, OUTPUT); 

    // Zet alle sensors als input
    for (int i = 0; i < _NUM_SENSORS; i++) {
        pinMode(LineSensor[i], INPUT);
    }

    //Pixel setup
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}


//-------------LOOP
void loop() {
    mazeLine();
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
void drive(int left, int right) { 
    analogWrite(MOTORA1, max(0, -left)); 
    analogWrite(MOTORA2, max(0, left)); 
    analogWrite(MOTORB1, max(0, -right)); 
    analogWrite(MOTORB2, max(0, right)); 
}

void stop() { 
    brakeLight();
    analogWrite(MOTORA1, 0); 
    analogWrite(MOTORA2, 0); 
    analogWrite(MOTORB1, 0); 
    analogWrite(MOTORB2, 0); 
} 

//-------------GRIPPER FUNCTIE
void gripper(int pulse) {
    static unsigned long timer;
    static int lastPulse;
    if (millis() > timer) {
        if (pulse > 0) {
            lastPulse = pulse;
        } else {
            pulse = lastPulse;
        }
        digitalWrite(GRIPPER, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER, LOW);
        timer = millis() + 20; //20ms interval voor servo
    }
}

//-------------GRIPPER OPEN
void gripperOpen() {
    gripper(1800);
}

//-------------GRIPPER SLUITEN
void gripperClosed() {
    gripper(1050);
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

//-------------Maze Line Follower
void mazeLine() {
    int sensorReadings[_NUM_SENSORS]; 
    int sum = 0; 
    
    for (int i = 0; i < _NUM_SENSORS; i++) { 
        sensorReadings[i] = analogRead(LineSensor[i]);
        sum += sensorReadings[i]; 
    }
    
    int average = sum / _NUM_SENSORS;
    _DEADZONELOW = average - 50;
    _DEADZONEHIGH = average + 50;
    int currentDirection = 0;

    if (sensorReadings[6] >= _DEADZONEHIGH && sensorReadings[7] >= _DEADZONEHIGH) { 
        currentDirection = -1;
        regularLight();
        drive(FULLSPEED, -120);
    }
    else if (sensorReadings[3] >= _DEADZONEHIGH && sensorReadings[4] >= _DEADZONEHIGH) { 
        currentDirection = 0;
        regularLight();
        drive(STEADY_SPEED, STEADY_SPEED);
    }
    else if (sensorReadings[0] >= _DEADZONEHIGH && sensorReadings[1] >= _DEADZONEHIGH) { 
        currentDirection = 1;
        regularLight();
        drive(-120, FULLSPEED);
    }
    else if (sensorReadings[4] >= _DEADZONEHIGH && sensorReadings[5] >= _DEADZONEHIGH) { 
        currentDirection = 3;
        regularLight();
        drive(STEADY_SPEED, 165);
    }
    else if (sensorReadings[5] >= _DEADZONEHIGH && sensorReadings[6] >= _DEADZONEHIGH) { 
        currentDirection = 4;
        blinkerRight();
        drive(STEADY_SPEED, 35);
    }
    else if (sensorReadings[2] >= _DEADZONEHIGH && sensorReadings[3] >= _DEADZONEHIGH) { 
        currentDirection = 5;
        regularLight();
        drive(165, STEADY_SPEED);
    }
    else if (sensorReadings[1] >= _DEADZONEHIGH && sensorReadings[2] >= _DEADZONEHIGH) { 
        currentDirection = 6;
        blinkerLeft();
        drive(35, STEADY_SPEED);
    }
    else if (sum < _DEADZONELOW * _NUM_SENSORS) { 
        currentDirection = 7;
        regularLight();
        drive(-255, 255);
    }

    if (currentDirection != _lastDirection) {
    _lastDirection = currentDirection;

    switch (currentDirection) {
        case 0: Serial.println("Midden op lijn -> rechtdoor!"); break;
        case 1: Serial.println("Links veel lijn -> draai rechts!"); break;
        case -1: Serial.println("Rechts veel lijn -> draai links!"); break;
        case 3: Serial.println("Iets rechts -> stuur beetje bij!"); break;
        case 4: Serial.println("Meer rechts -> stuur sterker bij!"); break;
        case 5: Serial.println("Iets links -> stuur beetje bij!"); break;
        case 6: Serial.println("Meer links -> stuur sterker bij!"); break;
        case 7: Serial.println("Geen lijn -> draai zoeken..."); break;
        }
    }
    delay(50);
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
    strip.setPixelColor(2, strip.Color(100, 100, 100));  // white
    strip.setPixelColor(3, strip.Color(100, 100, 100));  // white
    strip.show();
}