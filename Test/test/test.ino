#include <Arduino.h>

// Motor pins
#define MOTOR_LEFT_FORWARD 11
#define MOTOR_LEFT_BACKWARD 10
#define MOTOR_RIGHT_FORWARD 6
#define MOTOR_RIGHT_BACKWARD 5

// Sensor pins
#define SENSOR_0 A0
#define SENSOR_1 A1
#define SENSOR_2 A2
#define SENSOR_3 A3
#define SENSOR_4 A4
#define SENSOR_5 A5
#define SENSOR_6 A6
#define SENSOR_7 A7

#define _NUM_SENSORS 8
#define _DEADZONE_OFFSET 50

int LineSensor[_NUM_SENSORS] = {SENSOR_0, SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6, SENSOR_7};
int _DEADZONELOW = 0;
int _DEADZONEHIGH = 0;

// PID parameters
float Kp = 2.0;   // Verlaagde Proportional gain
float Ki = 0.1;   // Integral gain
float Kd = 1.0;   // Verlaagde Derivative gain

float previousError = 0;
float integral = 0;

void setup() {
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
    
    for (int i = A0; i <= A7; i++) {
        pinMode(i, INPUT);
    }
    
    Serial.begin(9600);
}

void loop() {
    int sensorValues[_NUM_SENSORS];
    int sum = 0;
    
    for (int i = 0; i < _NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(LineSensor[i]);
        sum += sensorValues[i];
    }
    
    int average = sum / _NUM_SENSORS;
    _DEADZONELOW = average - _DEADZONE_OFFSET;
    _DEADZONEHIGH = average + _DEADZONE_OFFSET;
    
    // Bereken de fout op basis van de sensorwaarden
    int error = calculateError(sensorValues);
    
    // PID-controller
    integral += error;
    float derivative = error - previousError;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;
    
    int baseSpeed = 200; // Aangepaste basis snelheid
    int leftSpeed = constrain(baseSpeed - correction, 0, 255);
    int rightSpeed = constrain(baseSpeed + correction, 0, 255);
    
    if (sensorValues[3] > _DEADZONEHIGH || sensorValues[4] > _DEADZONEHIGH) { // Midden sensoren zien de lijn
        moveForward(leftSpeed, rightSpeed);
    } else if (sensorValues[5] > _DEADZONEHIGH || sensorValues[6] > _DEADZONEHIGH || sensorValues[7] > _DEADZONEHIGH) { // Linker sensoren zien de lijn
        turnLeft();
    } else if (sensorValues[0] > _DEADZONEHIGH || sensorValues[1] > _DEADZONEHIGH || sensorValues[2] > _DEADZONEHIGH) { // Rechter sensoren zien de lijn
        turnRight();
    } else { // Lijn kwijt
        stopMotors();
    }
    
    delay(5); // Verlaagde delay voor snellere respons
}

int calculateError(int sensorValues[]) {
    // Gewogen foutberekening op basis van de sensorwaarden
    int error = 0;
    for (int i = 0; i < _NUM_SENSORS; i++) {
        error += (sensorValues[i] > _DEADZONEHIGH ? 1 : 0) * (3.5 - i);
    }
    return error;
}

void moveForward(int leftSpeed, int rightSpeed) {
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void turnLeft() {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 200); // Aangepaste snelheid
    analogWrite(MOTOR_LEFT_BACKWARD, 200); // Aangepaste snelheid
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void turnRight() {
    analogWrite(MOTOR_LEFT_FORWARD, 200); // Aangepaste snelheid
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 200); // Aangepaste snelheid
}

void stopMotors() {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}