#include <MsTimer2.h>

#define motor1 5
#define motor2 6
#define motor3 9
#define motor4 10

#define LED_ 13
#define SW_R 7
#define SW_B 8

#define Encoder_L 2
#define Encoder_R 3

int estep_L=0;
int estep_R=0;
int estep_readL=0;
int estep_readR=0;

int a1=0;

float Pv=1.8;
float Iv=3.2;
float Dv=2.5;

float err_P;
float err_I;
float err_D;
float err_B;

int goal=0;
int PID_val;

int test1=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  pinMode(motor3,OUTPUT);
  pinMode(motor4,OUTPUT);

  pinMode(LED_, OUTPUT);
  pinMode(SW_R, INPUT_PULLUP);
  pinMode(SW_B, INPUT_PULLUP);

  attachInterrupt(0,encoderL,FALLING);
  attachInterrupt(1,encoderR,FALLING);

  MsTimer2::set(200, t_intrrupt);
  MsTimer2::start();
}

void loop() {
  if(digitalRead(SW_R)==0){
    test1++;
    Serial.print("test : ");
    Serial.println(test1);
    delay(500);
  }
  else if(digitalRead(SW_B)==0){
    test1--;
    if(test1<=0) test1=0;
    Serial.print("test : ");
    Serial.println(test1);
    delay(500);
  }
  else{
    
  }
  PID_motor(test1);
  Serial.println(PID_val);
  delay(300);
}

void PID_motor(int m1){
    goal=m1;
}

void motor_controll(int a, int b, int m1_speed, int m2_speed){
  if(a!=0){
      analogWrite(motor1,m1_speed);
      analogWrite(motor2,0);
    }
  else{
      analogWrite(motor1,0);
      analogWrite(motor2,m1_speed);
    }
    
  if(b!=0){
      analogWrite(motor3,m2_speed);
      analogWrite(motor4,0);
    }
  else{
      analogWrite(motor3,0);
      analogWrite(motor4,m2_speed);
    }
}

void encoderL(){
  estep_L++;
}

void encoderR(){
  estep_R++;
}

void encoder_read(){
  estep_readL=estep_L;
  estep_L=0;
  estep_readR=estep_R;
  estep_R=0;
}

void pid_control(){
  err_P=estep_readL-goal;
  err_I+=err_P;
  err_D=err_B-err_P;
  err_B=err_P;
  PID_val=((err_P*Pv)+(err_I*Iv)+(err_D*Dv));
  if(PID_val>=255) PID_val=255;
  if(PID_val<=-255) PID_val=-255;
  motor_controll(0,0,abs(PID_val),0);
}

void t_intrrupt()
{
  encoder_read();
  pid_control();
}
