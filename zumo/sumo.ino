#include <Servo.h>
#define MID_SPEED 100
#define HIGH_SPEED 120
#define LOW_SPEED 80
#define TURN_SPEED 80

#define speedPinR 9           //  Front Wheel PWM pin connect Model-Y M_B ENA
#define RightMotorDirPin1 22  //Front Right Motor direction pin 1 to Model-Y M_B IN1  (K1)
#define RightMotorDirPin2 24  //Front Right Motor direction pin 2 to Model-Y M_B IN2   (K1)
#define LeftMotorDirPin1 26   //Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
#define LeftMotorDirPin2 28   //Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define speedPinL 10          //  Front Wheel PWM pin connect Model-Y M_B ENB

#define speedPinRB 11         //  Rear Wheel PWM pin connect Left Model-Y M_A ENA
#define RightMotorDirPin1B 5  //Rear Right Motor direction pin 1 to Model-Y M_A IN1 ( K1)
#define RightMotorDirPin2B 6  //Rear Right Motor direction pin 2 to Model-Y M_A IN2 ( K1)
#define LeftMotorDirPin1B 7   //Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3)
#define LeftMotorDirPin2B 8   //Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define speedPinLB 12         //  Rear Wheel PWM pin connect Model-Y M_A ENB

#define RightObstacleSensor 2  //Right obstacle sensor to D2 (front direction is from arduino point to voltage meter)
#define LeftObstacleSensor 3   //Left obstacle sensor to D3

#define sensor1 A4  // Left most sensor
#define sensor2 A3  // 2nd Left   sensor
#define sensor3 A2  // center sensor
#define sensor4 A1  // 2nd right sensor// Right most sensor
#define sensor5 A0  // Right most sensor

// sensores color
#define sensor6 A4   // Left most sensor
#define sensor7 A3   // 2nd Left   sensor
#define sensor8 A2   // center sensor
#define sensor9 A1   // 2nd right sensor// Right most sensor
#define sensor10 A0  // Right most sensor

/*motor control*/
void go_advance(int speed) {
  RL_fwd(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_fwd(speed);
}
void go_back(int speed) {
  RL_bck(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_bck(speed);
}
void left_turn(int speed) {
  RL_bck(0);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(0);
}
void right_turn(int speed) {
  RL_fwd(speed);
  RR_bck(0);
  FR_bck(0);
  FL_fwd(speed);
}

void sharpRightTurn(int speed_left,int speed_right)
{
   RL_fwd(speed_left);
   RR_bck(speed_right);
   FR_bck(speed_right);
   FL_fwd(speed_left); 
}
void sharpLeftTurn(int speed_left,int speed_right){
   RL_bck(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_bck(speed_left); 
}

void FR_fwd(int speed)  //front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  analogWrite(speedPinR, speed);
}
void FR_bck(int speed)  // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  analogWrite(speedPinR, speed);
}
void FL_fwd(int speed)  // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, speed);
}
void FL_bck(int speed)  // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, speed);
}

void RR_fwd(int speed)  //rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  analogWrite(speedPinRB, speed);
}
void RR_bck(int speed)  //rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  analogWrite(speedPinRB, speed);
}
void RL_fwd(int speed)  //rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
  analogWrite(speedPinLB, speed);
}
void RL_bck(int speed)  //rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
  analogWrite(speedPinLB, speed);
}

void stop_Stop()  //Stop
{
  analogWrite(speedPinLB, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
}


//Pins initialize
void init_GPIO() {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);
  pinMode(RightObstacleSensor, INPUT);
  pinMode(LeftObstacleSensor, INPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);
  pinMode(sensor7, INPUT);
  pinMode(sensor8, INPUT);
  pinMode(sensor9, INPUT);
  pinMode(sensor10, INPUT);

  stop_Stop();
}
void sumo_fight() {
  int IRvalueLeft = digitalRead(RightObstacleSensor);
  int IRvalueRight = digitalRead(LeftObstacleSensor);
  Serial.println("R sensorValue");
  Serial.println(IRvalueLeft);
  // delay(1000);
  Serial.println("L sensorValue");
  Serial.println(IRvalueRight);
  // delay(1000);

  String senstr = "";
  int s0 = !digitalRead(sensor1);
  int s1 = !digitalRead(sensor2);
  int s2 = !digitalRead(sensor3);
  int s3 = !digitalRead(sensor4);
  int s4 = !digitalRead(sensor5);
  int sensorvalue = 32;
  sensorvalue += s0 * 16 + s1 * 8 + s2 * 4 + s3 * 2 + s4;
  senstr = String(sensorvalue, BIN);
  senstr = senstr.substring(1, 6);

  Serial.print(senstr);
  Serial.print("\t");

  String senstrC = "";
  int s5 = !digitalRead(sensor6);
  int s6 = !digitalRead(sensor7);
  int s7 = !digitalRead(sensor8);
  int s8 = !digitalRead(sensor9);
  int s9 = !digitalRead(sensor10);
  int sensorvalueC = 32;
  sensorvalueC += s0 * 16 + s1 * 8 + s2 * 4 + s3 * 2 + s4;
  senstrC = String(sensorvalueC, BIN);
  senstrC = senstr.substring(1, 6);

  Serial.print(senstrC);
  Serial.print("\t");

  if (IRvalueLeft == LOW && IRvalueRight == LOW && senstrC == "11111") {
    //both sensor detected obstacle, go ahead
    go_advance(HIGH_SPEED); 
  } else if (IRvalueLeft == HIGH && IRvalueRight == HIGH) {
    //no obstacle
    // stop_Stop();
    sharpRightTurn(MID_SPEED, MID_SPEED);

  } else if (IRvalueLeft == LOW && IRvalueRight == HIGH) {
    //only left sensor detect obstacle
    left_turn(TURN_SPEED);  //!!!revisar
  } else if (IRvalueLeft == HIGH && IRvalueRight == LOW) {
    //only right sensor detect obstacle
    right_turn(TURN_SPEED); //!!!revisar
  }

// detección de línea blanca
  if (senstrC == "00001" || senstrC == "00011" || senstrC == "00111" || senstrC == "01111" || senstrC == "00110") {
    right_turn(LOW_SPEED);
    // delay(1000);
    stop_Stop();
  }

  if (senstrC == "11100" || senstrC == "11110" || senstrC == "11000" || senstrC == "10000" ) {
    left_turn(LOW_SPEED);
    // delay(1000);
    stop_Stop();
  }

  if (senstrC == "00000" || senstrC == "01110" || senstrC == "00110" || senstrC == "01100") {
    go_back(MID_SPEED);
    // delay(1000);
    stop_Stop();
  }
}


void setup() {
  init_GPIO();
  Serial.begin(9600);
}

void loop() {
  sumo_fight();
}