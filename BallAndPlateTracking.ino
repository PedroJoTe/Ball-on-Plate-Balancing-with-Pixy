/*

Program Dynamic Traffic Light ini dibuat menggunakan Camera Pixy cmucam5
dan pustaka bawaannya (http://www.cmucam.org/)
Dalam kode komputer ini di asumsikan ada 4 lajur pada satu simpangan,
dengan arah lampu dimulai dari barat searah jarum jam
mikrokontroller yang digunakan adalah arduino Mega
Copenhagen, Desember 2018 
Copyright@PDKM 2018

*/
#include <SPI.h>  
#include <Pixy.h>
#include <Servo.h>
#include<PID_v1.h>

int xSPin = D4;
int ySPin = D3;  

int xmaxi = 260;
int xmini = 56;
int ymaxi = 173;
int ymini = 25;
int j;
int xkalibrator = 85; // Servo kuning 80
int ykalibrator = 90; // Servo Merah

int xSP[] = {90, 90, 210, 210};
int ySP[] = {64, 145, 145, 64};


double xSetpoint, xInput, xOutput, xServoSignal; 
double ySetpoint, yInput, yOutput, yServoSignal; 

float xKp = 0.9;   // 1.1    
float xKi = 0.03;  //.05  
float xKd = 0.3;   //.2
float yKp = 0.5;   //.9
float yKi = 0.05;  // .05
float yKd = 0.3;   //.2

PID xmyPID(&xInput, &xOutput, &xSetpoint, xKp, xKi, xKd, DIRECT);                                                               
PID ymyPID(&yInput, &yOutput, &ySetpoint, yKp, yKi, yKd, DIRECT);                                                                                                                                                                                                        

Servo xmyServo;
Servo ymyServo;
Pixy nyaku;

void setup() {
  xSetpoint = 160;//  Sumbu X
  ySetpoint = 100; // sumbu Y
  
  Serial.begin(115200);
  xmyServo.attach(xSPin);
  ymyServo.attach(ySPin); 
  nyaku.init();
  Serial.print("Siap...\n");
                                    
  xmyPID.SetMode(AUTOMATIC);                                   
  xmyPID.SetOutputLimits(-70,70);
  ymyPID.SetMode(AUTOMATIC);                                   
  ymyPID.SetOutputLimits(-70,70);
}

void loop() {
  static int i = 0;
  uint16_t blocks;
  char buf[32];  
  blocks = nyaku.getBlocks();
    if (blocks) {
       i++;
      if (i%1==0) {
           for (j=0; j<blocks; j++) {           
              loopa();
           }        
       }
    } 
}

void loopa(){
  if (nyaku.blocks[j].x <= xmaxi) {
      xInput = nyaku.blocks[j].x;
      yInput = nyaku.blocks[j].y;
      if (xInput > xmaxi) {
        xInput = xmaxi;
      }
      if (xInput < xmini){
        xInput = xmini;
      }
                                 
      xmyPID.Compute();                    
      xServoSignal=xkalibrator+xOutput; // +/- tergatung posisi tungkai pd servo
      xmyServo.write(xServoSignal); 
      ymyPID.Compute();                    
      yServoSignal=ykalibrator+yOutput; // +/- tergatung posisi tungkai pd servo
      ymyServo.write(yServoSignal);     
      
      Serial.println("Posisi x dan y : ");
      Serial.print(xInput);
      Serial.print(",");
      Serial.print(yInput);
      Serial.print(",");
      Serial.print(xServoSignal);
      Serial.print(",");
      Serial.println(yServoSignal);

   } 
}

