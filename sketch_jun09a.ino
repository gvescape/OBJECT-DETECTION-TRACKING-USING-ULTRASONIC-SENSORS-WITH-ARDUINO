#include <Servo.h>
#define Rtrig 5
#define Recho 4
#define Ltrig 8
#define Lecho 7
#define Ctrig 10
#define Cecho 11
#define laser 6
Servo servo;
int angle = 90;
//Ultrasonic ultrasonicR(Rpino_trigger, Rpino_echo);
//Ultrasonic ultrasonicC(Cpino_trigger, Cpino_echo);
//Ultrasonic ultrasonicL(Lpino_trigger, Lpino_echo);
void setup()
{
  pinMode(laser,OUTPUT);
  pinMode(Ltrig,OUTPUT);
  pinMode(Rtrig,OUTPUT);
  pinMode(Ctrig,OUTPUT);
  pinMode(Lecho,INPUT);
  pinMode(Recho,INPUT);
  pinMode(Cecho,INPUT);
  servo.attach(9);
  servo.write(angle);
  Serial.begin(9600);
  Serial.println("reading values from sensor...");
}
 
void loop()
{
  int R = measure_distance_cmR();
  int L = measure_distance_cmL();
  int C = measure_distance_cmC(); 
  //cmMsecR = ultrasonicR.convert(microsecR, Ultrasonic::CM);
  //inMsecR = ultrasonicR.convert(microsecR, Ultrasonic::IN);
  //cmMsecL = ultrasonicL.convert(microsecL, Ultrasonic::CM);
  //inMsecL = ultrasonicL.convert(microsecL, Ultrasonic::IN);
  //cmMsecC = ultrasonicL.convert(microsecC, Ultrasonic::CM);
  //inMsecC = ultrasonicL.convert(microsecC, Ultrasonic::IN);
  //Exibe informacoes no serial monitor
  Serial.print("DistanceR: ");
  Serial.println(R);
  //delay(500);
  Serial.print("DistanceL: ");
  Serial.println(L);
  //delay(500);
  Serial.print("DistanceC: ");
  Serial.println(C);
  //delay(500);
  //Serial.print(",   angle: ");
  //Serial.println(angle);
  // Serial.print(" - Distancia em polegadas: ");
  // Serial.println(inMsecR);
  // delay(500);
  // sÃ³ ativa quando existe algum objeto a uma distÃ¢ncia < 50cm, em qualquer olho
    if ((R < 20) || (L < 20) || (C < 20)) 
    {
     if (((R > 20) && (L < 20) && (C < 20))||((R > 20) && (L < 20) && (C > 20))) 
     {
      if (angle<=180)
           {angle = angle + 5;
           servo.write(angle);} 
     }
      if ((R > 20) && (L > 20) && (C < 20))
     {
      digitalWrite(laser,HIGH);
      delay(20);
      digitalWrite(laser,LOW);       
     }
          // objeto mais a direita, vira para a direita]'P.
     if (((R < 20) && (L > 20) && (C < 20)) ||((R < 20) && (L > 20) && (C > 20))) 
    {
      if(angle>=0)
          {angle = angle - 5;
          servo.write(angle);}
     }
  }
  delay(30);
}

float measure_distance_cmR()
{
  float Rdistance =0;
  long Rtime_value=0;
  digitalWrite(Rtrig,HIGH);
  delayMicroseconds(10);
  digitalWrite(Rtrig,LOW);
  Rtime_value=pulseIn(Recho,HIGH);
  Rdistance=0.033*Rtime_value/2;
  return Rdistance;
    }
float measure_distance_cmC()
{
  float Cdistance =0;
  long Ctime_value=0;
  digitalWrite(Ctrig,HIGH);
  delayMicroseconds(10);
  digitalWrite(Ctrig,LOW);
  Ctime_value=pulseIn(Cecho,HIGH);
  Cdistance=0.033*Ctime_value/2;
  return Cdistance;
    }
float measure_distance_cmL()
{
  float Ldistance =0;
  long Ltime_value=0;
  digitalWrite(Ltrig,HIGH);
  delayMicroseconds(10);
  digitalWrite(Ltrig,LOW);
  Ltime_value=pulseIn(Lecho,HIGH);
  Ldistance=0.033*Ltime_value/2;
  return Ldistance;
    }
