#include <Shift595.h>
#include <Servo.h>
#include <enc28j60.h>
#include <etherShield.h>
#include <ip_arp_udp_tcp.h>
#include <net.h>
#include <ETHER_28J60.h>

//
// шаговые
// управляющие пины шаговых двигателей
#define   dataPin          7      // pin 14 on the 74HC595
#define   latchPin          4      // pin 12 on the 74HC595
#define   clockPin          3      // pin 11 on the 74HC595
// сдвиговых регистров в цепи
#define   numOfRegisters    1      // number of shift registers present
// инициализация
Shift595 Shifter(dataPin, latchPin, clockPin, numOfRegisters);
// оборотов за одно действие
int Steps = 256; 
int cstep = 0;

int motor1Pins[4] = {0,1,2,3};
int motor2Pins[4] = {4,5,6,7};
enum direction{
  forward=0,
  back=1
};

Servo servo1, servo2;

static uint8_t mac[6] = {0x54, 0x55, 0x56, 0x10, 0x00, 0x24};
static uint8_t ip[4] = {192,168,0,110};
static uint16_t port = 81;
int outputPin = 8;
ETHER_28J60 eth;

// подсветка камеры
const int camera_lights_pin = 9; // pin на котором загорается подсветка камеры
static unsigned long last_action = 0; // время последней активности
const unsigned long turn_off_backlight = 5000; // таймер выключения подсветки камеры в мс и равняется времени просмотра

void setup() {
  // установка timer one на вызов функции 1 раз в секунду
  pinMode(camera_lights_pin, OUTPUT);
  //Timer1.initialize(1000000); // инициализация таймера, 1000000 = 1s
  //Timer1.attachInterrupt(CallOneTimer); // установка функции обработчика TimerOne
  // инициализация ethernet
  eth.setup(mac, ip, port);
  // включение serial порта
  Serial.begin(9600);

  servo1.attach(5);
  servo2.attach(6);
  // установка положения по умолчанию на двигателях держателя камеры
  servo1.write(100);
  servo2.write(100);
  
  pinMode(outputPin, OUTPUT);

  // инициализация таймера 0
  OCR0A = 0x7F;
  TIMSK0 |= _BV(OCIE0A);
  // установка нулей на сдвигом регистре
  resetShiftRegister();
}

char html[] = "<h1 style='color:#551111;'>12345</h1>";
void loop() {
  /*
  // put your main code here, to run repeatedly: http://192.168.0.110:81/start
  //https://ru.ivideon.com/my
  //http://192.168.0.104:8080/
  //http://176.8.213.163:8080/video
  if(eth.serviceRequest()){
    //Serial.println("+");
    eth.print(html);
    eth.respond();
  }*/

  
  char* params;
  char h1[] = "<H1>hello</H1>";
  char img[] = "<img src='http://176.8.213.163:8080/video' width='500px' /><br />";
  char buttons[] = "<meta name='viewport' content='width=device-width, initial-scale=1'><a href='http://192.168.0.110:81/start'>start</a> <a href='http://192.168.0.110:81/stop'>stop</a><hr /> <a href='http://192.168.0.110:81/s1down'>s1down</a><a href='http://192.168.0.110:81/s1up'>s1up</a><a href='http://192.168.0.110:81/s2down'>s2down</a><a href='http://192.168.0.110:81/s2up'>s2up</a><hr />";
  if (params = eth.serviceRequest())
  {
    
    //eth.print(h1);
    //eth.print(img);
    //eth.print(buttons);
    //eth.print(params);
    //if (strcmp(params, "?cmd=on1") == 0){digitalWrite(outputPin, HIGH);}
    if (strcmp(params, "stop") == 0){digitalWrite(outputPin, LOW); ActivatedAction();}
    if (strcmp(params, "start") == 0){digitalWrite(outputPin, HIGH); ActivatedAction();}

    if (strcmp(params, "s1down") == 0){
      unsigned int current_angle_s1;
      unsigned int new_angle_s1;
      current_angle_s1 = servo1.read();
      new_angle_s1 = current_angle_s1-10;
      Serial.println(new_angle_s1);
      if(new_angle_s1<=100){new_angle_s1=100;}      
      servo1.write(new_angle_s1); 
      ActivatedAction();
      }
    if (strcmp(params, "s1up") == 0){
      unsigned int current_angle_s1;
      unsigned int new_angle_s1;
      current_angle_s1 = servo1.read();
      new_angle_s1 = current_angle_s1+10;
      Serial.println(new_angle_s1);
      if(new_angle_s1>=180){new_angle_s1=180;}      
      servo1.write(new_angle_s1); 
      ActivatedAction();
      }
    if (strcmp(params, "s2down") == 0){
      unsigned int current_angle_s2;
      unsigned int new_angle_s2;
      current_angle_s2 = servo2.read();
      new_angle_s2 = current_angle_s2-10;
      Serial.println(new_angle_s2);
      if(new_angle_s2<=20){new_angle_s2=20;}      
      servo2.write(new_angle_s2); 
      ActivatedAction();
      }
    if (strcmp(params, "s2up") == 0){
      unsigned int current_angle_s2;
      unsigned int new_angle_s2;
      current_angle_s2 = servo2.read();
      new_angle_s2 = current_angle_s2+10;
      Serial.println(new_angle_s2);
      if(new_angle_s2>=160){new_angle_s2=160;}      
      servo2.write(new_angle_s2); 
      ActivatedAction();
      }
    // stepper 1
    if (strcmp(params, "motor1left") == 0){
      for(int x=0;x<Steps;x++)
      {
        step1(motor1Pins, true);
      }      
      resetShiftRegister();
      ActivatedAction();
      }
    if (strcmp(params, "motor1right") == 0){
      for(int x=0;x<Steps;x++)
      {
        step1(motor1Pins, false);
      }      
      resetShiftRegister();
      ActivatedAction();
      }
    // stepper 2
    if (strcmp(params, "motor2left") == 0){
      for(int x=0;x<Steps;x++)
      {
        step1(motor2Pins, true);
      }      
      resetShiftRegister();
      ActivatedAction();
      }
    if (strcmp(params, "motor2right") == 0){
      for(int x=0;x<Steps;x++)
      {
        step1(motor2Pins, false);
      }      
      resetShiftRegister();
      ActivatedAction();
      }
    eth.respond();
  }


  //servo1.write(0); //ставим вал под 0
  //delay(2000); //ждем 2 секунды
  //servo1.write(180); //ставим вал под 180
  //delay(2000); //ждем 2 секунды
}

void ActivatedAction() // запоминание последнего времени какого-либо действия  
{
  digitalWrite(camera_lights_pin, HIGH); 
  last_action = millis();
}

// обработчик прерывания таймера 0
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long new_time = millis();
  /*ПОДСВЕТКА*/
  // сравнение счетчика текущего времени с временем посденего какого-либо действия. если прошло более заданного времени, то подсветка отключается
  if((new_time-last_action)>=turn_off_backlight)
  {
    digitalWrite(camera_lights_pin, LOW);
    last_action = 0;    
  }
  //Serial.println("new_time: "); Serial.print(new_time); Serial.print(" last_action: "); Serial.print(last_action); Serial.println("");
}

void step1(int motorPins[], bool dir )
{
  //stepp
  switch(cstep)
  {
   case 0:
     Shifter.setRegisterPin(motorPins[0], LOW); 
     Shifter.setRegisterPin(motorPins[1], LOW);
     Shifter.setRegisterPin(motorPins[2], LOW);
     Shifter.setRegisterPin(motorPins[3], HIGH);
   break; 
   case 1:
     Shifter.setRegisterPin(motorPins[0], LOW); 
     Shifter.setRegisterPin(motorPins[1], LOW);
     Shifter.setRegisterPin(motorPins[2], HIGH);
     Shifter.setRegisterPin(motorPins[3], HIGH);
   break; 
   case 2:
     Shifter.setRegisterPin(motorPins[0], LOW); 
     Shifter.setRegisterPin(motorPins[1], LOW);
     Shifter.setRegisterPin(motorPins[2], HIGH);
     Shifter.setRegisterPin(motorPins[3], LOW);
   break; 
   case 3:
     Shifter.setRegisterPin(motorPins[0], LOW); 
     Shifter.setRegisterPin(motorPins[1], HIGH);
     Shifter.setRegisterPin(motorPins[2], HIGH);
     Shifter.setRegisterPin(motorPins[3], LOW);
   break; 
   case 4:
     Shifter.setRegisterPin(motorPins[0], LOW); 
     Shifter.setRegisterPin(motorPins[1], HIGH);
     Shifter.setRegisterPin(motorPins[2], LOW);
     Shifter.setRegisterPin(motorPins[3], LOW);
   break; 
   case 5:
     Shifter.setRegisterPin(motorPins[0], HIGH); 
     Shifter.setRegisterPin(motorPins[1], HIGH);
     Shifter.setRegisterPin(motorPins[2], LOW);
     Shifter.setRegisterPin(motorPins[3], LOW);
   break; 
     case 6:
     Shifter.setRegisterPin(motorPins[0], HIGH); 
     Shifter.setRegisterPin(motorPins[1], LOW);
     Shifter.setRegisterPin(motorPins[2], LOW);
     Shifter.setRegisterPin(motorPins[3], LOW);
   break; 
   case 7:
     Shifter.setRegisterPin(motorPins[0], HIGH); 
     Shifter.setRegisterPin(motorPins[1], LOW);
     Shifter.setRegisterPin(motorPins[2], LOW);
     Shifter.setRegisterPin(motorPins[3], HIGH);
   break; 
   default:
     Shifter.setRegisterPin(motorPins[0], LOW); 
     Shifter.setRegisterPin(motorPins[1], LOW);
     Shifter.setRegisterPin(motorPins[2], LOW);
     Shifter.setRegisterPin(motorPins[3], LOW);
   break; 
  }
   if(dir){
    cstep++;
   }else{
    cstep--;
   }
   if(cstep>7){
    cstep=0;
   }
   if(cstep<0){
    cstep=7;
   }
   delay(1);
}

void resetShiftRegister(){
  for(int p=0;p<8;p++){
      Shifter.setRegisterPin(p, LOW);
    }
}

