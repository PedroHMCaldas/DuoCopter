#include <Wire.h>
#include <Servo.h>
#include <Nextion.h>

#define PI 3.14159265358979323846

#define CONTROL_LOOP_RATE 5000  // Taxa do loop de controle 200 Hz
#define COMMUNICATION_LOOP_RATE 50  // Taxa do loop de comunicação 20 Hz

#define MotorsOFF 1000 
#define MotorsMaxSpeed 2000

#define RightMotorPin 2
#define LeftMotorPin 3
#define Relay1Pin 5
#define Relay2Pin 6

Servo leftMotor, rightMotor;

// Variáveis PID
double PID = 0.0, valP = 0.0, valI = 0.0, valD = 0.0;
double kp = 3.55, ki = 0.003, kd = 2.05;

// Variáveis dos motores
float leftMotorPWM = 0.0, rightMotorPWM = 0.0;
float error = 0.0, prevError = 0.0;
float angSet = 0.0;
const double basePWM = 1300.0;

// Variáveis de tempo
float time = 0.0, prevTime = 0.0, elpsdTime = 0.0;
unsigned long controlLoopStart = 0, communicationLoopStart = 0;

// Sensores
float Gyr_roll = 0.0, angRoll = 0.0;
const float rad_degConv = 180 / PI;

// Variáveis Nextion
//final de cada dado enviado para o display
String endChar = String(char(0xff)) + String(char(0xff)) + String(char(0xff));

// Elementos da página 1 - home
NexPage home = NexPage(1, 0, "home");

// Elementos da página 3 - menu de controle
//botões
NexDSButton btOnOff = NexDSButton(3, 1, "bt0");
NexButton backButton = NexButton(3, 6, "b4");
NexButton applySetAng = NexButton(3, 4, "b2");
NexButton applyPulse = NexButton(3, 19, "b8");
NexDSButton applyStep = NexDSButton(3, 20, "bt1");

//variáveis
NexVariable setAngle = NexVariable(3, 14, "angSet");
NexVariable pulse = NexVariable(3, 25, "pulse");
NexVariable step = NexVariable(3, 26, "step");

// Elementos da página 4 - menu de ajuste PID

//botões
NexButton applyPID = NexButton(4, 3, "b2");

//variáveis
NexVariable setKp = NexVariable(4, 20, "kp");
NexVariable setKi = NexVariable(4, 21, "ki");
NexVariable setKd = NexVariable(4, 22, "kd");

// Lista de varredura dos botões, eles serão checados se foram pressionados nesta ordem
NexTouch *nex_listen_list[] = {
  &btOnOff,
  &backButton,
  &applySetAng,
  &applyPulse,
  &applyStep,
  &applyPID,
  NULL
};

// Declara variáveis do display Nextion
int32_t setAngIHM;
uint32_t setKpIHM;
uint32_t setKiIHM;
uint32_t setKdIHM;
uint32_t pulseValue;
uint32_t stepValue;

uint32_t duoOn;
uint32_t stepOn;
bool pulseOn;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Configuração da MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); Wire.write(0);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C); Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); Wire.write(0x08);
    Wire.endTransmission();

    // Configuração dos motores e relés
    rightMotor.attach(RightMotorPin);
    leftMotor.attach(LeftMotorPin);
    digitalWrite(Relay1Pin, HIGH);
    digitalWrite(Relay2Pin, HIGH);
    pinMode(Relay1Pin, OUTPUT);
    pinMode(Relay2Pin, OUTPUT);

    //setup IHM Nextion
    // para trocar a taxa interna do display é necessário iniciar a comunicação e trocar via comando serial
    Serial2.begin(9600);
    nexInit();
    delay(500);
    Serial2.print("baud=115200" + endChar);
    delay(500);
    Serial2.end();
    delay(500);
    Serial2.begin(115200);

    btOnOff.attachPush(call_btOnOff);   //define a função que será carregada com o botão pressionado
    backButton.attachPop(call_backButton);  //define a função que será carregada com o botão solto
    applySetAng.attachPop(call_applySetAng);
    applyPulse.attachPop(call_applyPulse);
    applyStep.attachPop(call_applyStep);
    applyPID.attachPush(call_applyPID);

    home.show();  //inicializa o menu home, após estabelecer comunicação serial, saindo da tela de boot
}
   

void loop() {
    unsigned long currentMicros = micros();
    unsigned long currentMillis = millis();

  //loop de controle
    if (currentMicros - controlLoopStart > CONTROL_LOOP_RATE) {
        controlLoopStart = currentMicros;

        // Condiciona o controle para a planta ligada
        if(duoOn){
        // Cálculo do tempo para o controlador e IMU
        prevTime = time;
        time = millis();
        elpsdTime = (time - prevTime) / 1000.0;

        // Condiciona a aplicação de um pulso na referência
        if(pulseOn){
          doPulse();
        }

        // Chama as funções principais de controle
        readAngle();
        doPID();
        writeMotors();
        }
    }

  //loop de comunicação
    if (currentMillis - communicationLoopStart > COMMUNICATION_LOOP_RATE) {
        communicationLoopStart = currentMillis;

        nexLoop(nex_listen_list); // Realiza a varredura na lista de botões, para verificar se algum botão foi pressionado

        // Condiciona a escrita no display para a planta ligada
        if(duoOn){
          // Chama a função para escrever na saida serial
          // writeSerial();
          // Chama a função para escrever na IHM
          writeIHM(); 
        }  
    }
}

// Função para ler o ângulo de roll da IMU
void readAngle() {
    // Solicita e guarda o valor do acelerômetro da IMU
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    int16_t Acc_rawX = Wire.read() << 8 | Wire.read();
    int16_t Acc_rawY = Wire.read() << 8 | Wire.read();
    int16_t Acc_rawZ = Wire.read() << 8 | Wire.read();

    // conversão para aceleração g e compensação da calibração
    float AccX = ((float)Acc_rawX / 4096.0) - 0.02;
    float AccY = (float)Acc_rawY / 4096.0;
    float AccZ = ((float)Acc_rawZ / 4096.0) + 0.05;

    // Solicita e guarda o valor do giroscópio da IMU
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2, true);
    int16_t Gyr_rawX = Wire.read() << 8 | Wire.read();

    float Acc_roll = atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * rad_degConv; // Cálculo do roll do acelerômetro
    
    // conversão para deg/s e compensação da calibração
    float GyrX = ((float)Gyr_rawX / 65.5) + 5.4506; 
    Gyr_roll = angRoll + (GyrX * elpsdTime); // Cálculo do roll do giroscopio
    angRoll = 0.98 * Gyr_roll + 0.02 * Acc_roll; // filtro complementar
}

// Função para cálculo do PID
void doPID() {
    angSet = constrain(angSet, -35, 35); // Limita o ângulo de entrada por segurança
    error = angRoll - angSet;
    valP = kp * error;
    valI = constrain(valI + (ki * error), -500, 500); // Constrain para anti-windup da integral
    valD = kd * ((error - prevError) / elpsdTime);
    PID = constrain(valP + valI + valD, -1000, 1000); // Constrain para evitar valores de PID fora do range
    prevError = error;
}

// Função para escrever nos motores
void writeMotors() {
    leftMotorPWM = constrain(basePWM + PID, MotorsOFF, MotorsMaxSpeed); // constrain para atender a calibração das ESCs
    rightMotorPWM = constrain(basePWM - PID, MotorsOFF, MotorsMaxSpeed);
    //Escreve nos motores
    leftMotor.writeMicroseconds(leftMotorPWM);
    rightMotor.writeMicroseconds(rightMotorPWM);
}

// Função para escrever na saída serial (formato para o Matlab)
void writeSerial() {
    Serial.print(pulseValue);
    Serial.print(F(","));
    Serial.println(angRoll);
}

void writeIHM() {
  static uint8_t i = 0;  
  static int angleIHM, angleWaveForm, angleAnimation;

  angleIHM = int(angRoll * 10); // Conversão de float para inteiro
  angleWaveForm = constrain(map(angleIHM, -500, 500, 0, 146), 0, 146); // mapeamento dos limites do gráfico
  angleAnimation = constrain(map(angleIHM, -350, 350, 49, 77), 49, 77);  // mapeamento dos limites dos frames da animação

  if (Serial2.availableForWrite() > 10) {  // Só envia se houver espaço no buffer
    switch (i) {
      case 0:
        Serial2.print("x1.val="); // retorna o ângulo
        Serial2.print(angleIHM);
        Serial2.print(endChar);
        break;

      case 1:
        Serial2.print("add 7,0,"); // Escreve no gráfico
        Serial2.print(angleWaveForm);
        Serial2.print(endChar);
        break;

      case 2:
        Serial2.print("p0.pic="); // Altera o frame da animação
        Serial2.print(angleAnimation);
        Serial2.print(endChar);
        break;
    }
    i = (i + 1) % 3;  // Ciclo entre 0, 1 e 2
  }
}

void doPulse() {
  static float angBkp;
  static int i;
  i++;

  if (i==1){
    angBkp = angSet; // Guarda o ângulo para retornar ao fim do pulso
    angSet += (float) pulseValue; // Aplica o pulso
  }

  // Retorna o ângulo após 50 iterações (a 100 Hz equivale a 0,5s)
  if (i==50){
    angSet = angBkp;
    i = 0;
    pulseOn = 0;
  }
}

// Funções de chamada dos botões

// botão liga e desliga
void call_btOnOff(void *ptr) {
  btOnOff.getValue(&duoOn); // Verifica o estado do botão

  if (duoOn) {
    digitalWrite(Relay1Pin, LOW); // Aciona os relés
    digitalWrite(Relay2Pin, LOW);
    leftMotor.writeMicroseconds(MotorsOFF); // Inicia com os motores desligados
    rightMotor.writeMicroseconds(MotorsOFF);
    delay(2000);
  } else {
    leftMotor.writeMicroseconds(MotorsOFF); // Desliga os motores
    rightMotor.writeMicroseconds(MotorsOFF);
    digitalWrite(Relay1Pin, HIGH); // Desliga os relés
    digitalWrite(Relay2Pin, HIGH);
  }
}

// botão de voltar
void call_backButton(void *ptr) {
  // desliga o sistema caso pressionado
  duoOn = 0;

  leftMotor.writeMicroseconds(MotorsOFF); //desliga os motores
  rightMotor.writeMicroseconds(MotorsOFF);
  digitalWrite(Relay1Pin, HIGH); //desliga os relés
  digitalWrite(Relay2Pin, HIGH);
}

// botão de aplicar ângulo
void call_applySetAng(void *ptr) {
  setAngle.getValue(&setAngIHM); // Verifica o valor setado

  angSet = (float) setAngIHM / 10.0; // Converte o inteiro do display para float
}

void call_applyPulse(void *ptr) {
  pulse.getValue(&pulseValue); // Verifica a intensidade do pulso a ser aplicado

  pulseOn = 1; // Ativa a condição para aplicar o pulso no loop de controle
}

void call_applyStep(void *ptr) {
  static float angBkp;
  step.getValue(&stepValue); // Verifica a intensidade do degrau
  applyStep.getValue(&stepOn); // Verifica o valor do botão

  if(stepOn){
    // Se o botão tiver sido ativado adiciona o degrau
    angBkp = angSet;
    angSet += (float) stepValue;
  }else{
    // Se o botão tiver sido desativado remove o degrau
    angSet = angBkp;
  }
}

void call_applyPID(void *ptr) {
  // Lê os valores de ganho setados no display
  setKp.getValue(&setKpIHM);
  setKi.getValue(&setKiIHM);
  setKd.getValue(&setKdIHM);

  // Converte os valores inteiros do display para float e adiciona às variáveis do controlador
  kp = (float) setKpIHM/1000.0;
  ki = (float) setKiIHM/1000.0;
  kd = (float) setKdIHM/1000.0;
}