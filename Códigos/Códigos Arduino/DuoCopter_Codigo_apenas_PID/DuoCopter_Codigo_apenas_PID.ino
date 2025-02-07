#include <Wire.h>
#include <Servo.h>

#define RightMotorPin 2
#define LeftMotorPin 3
#define Relay1Pin 5
#define Relay2Pin 6

Servo leftMotor, rightMotor;

// Variáveis PID
double PID = 0.0, valP = 0.0, valI = 0.0, valD = 0.0;
double kp = 3.851, ki = 0.031, kd = 1.75;

// Variáveis dos motores
float leftMotorPWM = 0.0, rightMotorPWM = 0.0;
float error = 0.0, prevError = 0.0;
float angSet = 0.0;
const double basePWM = 1300.0;

// Variáveis de tempo
float time = 0.0, prevTime = 0.0, elpsdTime = 0.0;
unsigned long controlLoopStart = 0, communicationLoopStart = 0;
unsigned long startTime = 0;

// Sensores
float Gyr_roll = 0.0, angRoll = 0.0;
const float rad_degConv = 180 / PI;

bool ativa = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    rightMotor.attach(RightMotorPin);
    leftMotor.attach(LeftMotorPin);
    pinMode(Relay1Pin, OUTPUT);
    pinMode(Relay2Pin, OUTPUT);
    digitalWrite(Relay1Pin, HIGH);
    digitalWrite(Relay2Pin, HIGH);

    rightMotor.writeMicroseconds(2000);
    leftMotor.writeMicroseconds(2000);
    delay(200);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    delay(2000);
    rightMotor.writeMicroseconds(1000);
    leftMotor.writeMicroseconds(1000);
    delay(5000);

    startTime = millis();
}

void loop() {
    unsigned long currentMillis = millis();

    // Controle PID
    if (micros() - controlLoopStart > 10000) {
        controlLoopStart = micros();
        prevTime = time;
        time = millis();
        elpsdTime = (time - prevTime) / 1000.0;
        readAngle();
        doPID();
        writeMotors();
    }

    // Comunicação Serial
    if (currentMillis - communicationLoopStart > 100) {
        communicationLoopStart = currentMillis;
        writeSerial();
    }

    if (Serial.available() > 0 && Serial.read() == 'D') {
        ativa = 1;
    }

    if (ativa == 1){
      // Aplicação de sinais de entrada
      //applyPulseTrain(25, 0.5, 5); // Pulso de 25 graus a cada 5 segundos com duração de 2 segundos
      // applySquareWave(0.125, 10, -20);  // Onda quadrada 0,125 Hz entre 10 e -20 graus
      applySineWave(0.5, 20); // Onda senoidal 0,5 Hz com amplitude de 20 graus
    }
}

void applyPulseTrain(float amplitude, float pulseDuration, float interval) {
    unsigned long timeSinceStart = (millis() - startTime) / 1000.0;
    if ((int)(timeSinceStart / interval) % 2 == 0 && fmod(timeSinceStart, interval) < pulseDuration) {
        angSet = amplitude;
    } else {
        angSet = 0;
    }
}

void applySquareWave(float frequency, float upperLimit, float lowerLimit) {
    int period = 1000.0 / frequency;  // Converte implicitamente para int
    if ((millis() / (period / 2)) % 2 == 0) {
        angSet = upperLimit;
    } else {
        angSet = lowerLimit;
    }
}

void applySineWave(float frequency, float amplitude) {
    float timeSeconds = (millis() - startTime) / 1000.0;
    angSet = amplitude * sin(2 * PI * frequency * timeSeconds);
}

void readAngle() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    int16_t Acc_rawX = Wire.read() << 8 | Wire.read();
    int16_t Acc_rawY = Wire.read() << 8 | Wire.read();
    int16_t Acc_rawZ = Wire.read() << 8 | Wire.read();

    float AccX = ((float)Acc_rawX / 4096.0) - 0.02;
    float AccY = (float)Acc_rawY / 4096.0;
    float AccZ = ((float)Acc_rawZ / 4096.0) + 0.05;

    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2, true);
    int16_t Gyr_rawX = Wire.read() << 8 | Wire.read();

    float Acc_roll = atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * rad_degConv;
    float GyrX = ((float)Gyr_rawX / 65.5) + 5.4506;
    Gyr_roll = angRoll + (GyrX * elpsdTime);
    angRoll = 0.98 * Gyr_roll + 0.02 * Acc_roll;
}

void doPID() {
    error = angRoll - angSet;
    valP = kp * error;
    valI = constrain(valI + (ki * error), -500, 500);
    valD = kd * ((error - prevError) / elpsdTime);
    PID = constrain(valP + valI + valD, -1000, 1000);
    prevError = error;
}

void writeMotors() {
    leftMotorPWM = constrain(basePWM + PID, 1000, 2000);
    rightMotorPWM = constrain(basePWM - PID, 1000, 2000);
    leftMotor.writeMicroseconds(leftMotorPWM);
    rightMotor.writeMicroseconds(rightMotorPWM);
}

void writeSerial() {
    Serial.print(angSet);
    Serial.print(F(","));
    Serial.println(angRoll);
}
