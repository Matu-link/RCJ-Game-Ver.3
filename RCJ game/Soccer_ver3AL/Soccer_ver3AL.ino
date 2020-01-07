#include <IRsensor.h>
#include <moving_average.h>
#include "math.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA219.h>
#define IR_NUM 8
//モータPIN番号(ハード依存)
#define M1a 4
#define M1b 5
#define M2a 6
#define M2b 7
#define M3a 8
#define M3b 9
#define M4a 2
#define M4b 3
#define echoPin1 23 // Echo Pin
#define trigPin1 22 // Trigger Pin
#define echoPin2 25 // Echo Pin
#define trigPin2 24 // Trigger Pin
#define buzzer 30
#define Sbutton 29

int val = 0; // 入力ピンの状態を記憶しておく用
int old_val = 0; // 前回の入力ピンの状態を記憶
int state = 0; // LEDの状態（0ならオフ、1ならオン）
int deg;                 // ボールの角度
int power1;             // 
int power2;
int power3;
int power4;
int x1a; //モータHIGH or LOW
int x1b;
int x2a;
int x2b;
int x3a;
int x3b;
int x4a;
int x4b;

double Duration1 = 0; //受信した間隔
double Distance1 = 0; //距離
double Duration2 = 0; 
double Distance2 = 0; 
// RoboCupJunior IR Ball waveform MODE-A T=833[us]
// https://www.elekit.co.jp/pdf/RCJ-05%20waveform_j.pdf

void ssw1(void){
  digitalWrite(trigPin1, LOW); 
  delayMicroseconds(2); 
  digitalWrite( trigPin1, HIGH );//超音波を出力
  delayMicroseconds( 10 ); //
  digitalWrite( trigPin1, LOW );
  Duration1 = pulseIn( echoPin1, HIGH );//センサからの入力
  if (Duration1 > 0 ) { 
    Duration1 = Duration1/2;
    Distance1 = Duration1*340*100/1000000;// 音速を340m/sに設定
   
  }
}
  void ssw2(void){
    digitalWrite(trigPin2, LOW); 
  delayMicroseconds(2); 
  digitalWrite( trigPin2, HIGH );//超音波を出力
  delayMicroseconds( 10 ); //
  digitalWrite( trigPin2, LOW );
  Duration2 = pulseIn( echoPin2, HIGH );//センサからの入力
  if (Duration2 > 0) { 
    Duration2 = Duration2/2;
    Distance2 = Duration2*340*100/1000000;// 音速を340m/sに設定
   
  }
  }












#define BNO055_SAMPLERATE_DELAY_MS (100)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(420);
}
void displaySensorStatus(void)
{
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(420);
}
void displayCalStatus(void){
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
}













Adafruit_INA219 ina219;


#define T_MODEA 833

MovingAverage smaForRadius(20);
MovingAverage smaForTheta(20);

unsigned long time_ms = 0;

void setup() {
    Serial.begin(115200);
    setAllSensorPinsInput();
      pinMode(Sbutton, INPUT_PULLUP);
      pinMode( echoPin1, INPUT );
      pinMode( trigPin1, OUTPUT );
      pinMode( echoPin2, INPUT );
      pinMode( trigPin2, OUTPUT );
      pinMode(buzzer,OUTPUT);
  while (!Serial) {
     
      delay(1);
     
       
  }

  uint32_t currentFrequency;
    
  
  ina219.begin();
  

  
      Serial.println("Orientation Sensor Test"); Serial.println("");
 
  /* Initialise the sensor */
   Serial.println("Orientation Sensor Test"); Serial.println("");
 
  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  displaySensorDetails();
  displaySensorStatus();
  bno.setExtCrystalUse(true);
}


void loop() {
 val = digitalRead(Sbutton);

  if ((val == HIGH) && (old_val == LOW)) {
    state = 1 - state;
    delay(50); // バウンシング解消用の遅延
  }
  old_val = val;
  Serial.print(val);
  Serial.print("\n");

sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
   displayCalStatus();
  Serial.println("");
  delay(BNO055_SAMPLERATE_DELAY_MS);


  
  
 
 
  

if(state == 1){
     Stop();
}


   if(state == 0){
    
    float           pulseWidth[IR_NUM]; // パルス幅を格納する変数
    sensorInfo_t    sensorInfo;         // 実験用の測定データ ベクトルのみ使う場合は無視することも可能
    vectorXY_t      vectorXY;           // 直交座標系のベクトル構造体
    vectorRT_t      vectorRT;           // 極座標系のベクトル構造体
    vectorRT_t      vectorRTWithSma;    // 移動平均を適用させた極座標系ベクトル

    sensorInfo  = getAllSensorPulseWidth(pulseWidth, T_MODEA);
    vectorXY    = calcVectorXYFromPulseWidth(pulseWidth);
    vectorRT    = calcRTfromXY(&vectorXY);

    vectorRTWithSma.theta   = smaForTheta.updateData(vectorRT.theta);
    vectorRTWithSma.radius  = smaForRadius.updateData(vectorRT.radius);

    // 50ms周期でシリアルプリント
   if (millis() - time_ms > 50) {
        time_ms = millis();
        
        serialPrintAllPusleWidth(pulseWidth, &sensorInfo);
        Serial.print("\t");
        serialPrintVectorXY(&vectorXY);
        Serial.print("\t");
        serialPrintVectorRT(&vectorRTWithSma);
        Serial.print("\t");
        Serial.print(millis());
        Serial.print("\n");
    }
     
      switch (sensorInfo.maxSensorNumber){
      case 1: (sensorInfo.maxSensorNumber == 4);
           deg = 0;
               power1 = 240;
              power2 = 240;
              power3 = 240;
              power4 = 240;
      case 2: (sensorInfo.maxSensorNumber == 5);
           deg = 45;
              power1 = 220;
              power2 = 220;
              power3 = 220;
              power4 = 220;
      case 3: (sensorInfo.maxSensorNumber == 6);
           deg = 90;
               power1 = 0;
              power2 = 180;
              power3 = 0;
              power4 = 180;
      case 4: (sensorInfo.maxSensorNumber == 7); 
           deg = 135;
              power1 = 180;
              power2 = 0;
              power3 = 180;
              power4 = 0;
      case 5: (sensorInfo.maxSensorNumber == 0);
           deg = 180;
               power1 = 180;
              power2 = 130;
              power3 = 180;
              power4 = 130;
      case 6: (sensorInfo.maxSensorNumber == 1);
           deg = 225;
              power1 = 80;
              power2 = 80;
              power3 = 80;
              power4 = 80;
       case 7: (sensorInfo.maxSensorNumber == 2);
          deg = 270;
              power1 = 180;
              power2 = 0;
              power3 = 180;
              power4 = 0;
       case 8: (sensorInfo.maxSensorNumber == 3);
          deg = 315;
              power1 = 220;
              power2 = 220;
              power3 = 220;
              power4 = 220;
         }


  if (sensorInfo.maxSensorNumber == 0) {
      x1a = 0;
      x1b = power1;
   }else if (sensorInfo.maxSensorNumber == 4) {
      x1a = power1;
      x1b = 0;
   }else if (sensorInfo.maxSensorNumber == 5) {
      x1a = power1;
      x1b = 0;
   } else if (sensorInfo.maxSensorNumber == 6) {
      x1a = 0;
      x1b = power1;
        }else if (sensorInfo.maxSensorNumber == 7) {
      x1a = 0;
      x1b = power1;
        }else if (sensorInfo.maxSensorNumber == 1) {
       x1a = 0;
      x1b = power1;
        }else if (sensorInfo.maxSensorNumber == 2) {
      x1a = 0;
      x1b = power1;
        }else if (sensorInfo.maxSensorNumber == 3) {
      x1a = 0;
      x1b = power1;
        }

    if (sensorInfo.maxSensorNumber == 0) {
      x2a = power2;
      x2b = 0;
        }
    else if (sensorInfo.maxSensorNumber == 4) {
      x2a = 0;
      x2b = power2;
       }else if (sensorInfo.maxSensorNumber == 5) {
      x2a = power2;
      x2b = 0;
       }
    else if (sensorInfo.maxSensorNumber == 6) {
      x2a = power2;
      x2b = 0;
       }
    else if (sensorInfo.maxSensorNumber == 7) {
       x2a = 0;
      x2b = power2;
       }else if (sensorInfo.maxSensorNumber == 1) {
      x2a = power2;
      x2b = 0;
       }else if (sensorInfo.maxSensorNumber == 2) {
      x2a = power2;
      x2b = 0;
       }else if (sensorInfo.maxSensorNumber == 3) {
      x2a = 0;
      x2b = power2;
       }
       
   if (sensorInfo.maxSensorNumber == 0) {
      x3a = power3;
      x3b = 0;
    }
   else if (sensorInfo.maxSensorNumber == 4) {
      x3a = 0;
      x3b = power3;
       }
    else if (sensorInfo.maxSensorNumber == 5) {
      x3a = 0;
      x3b = power3;
       }
    else if (sensorInfo.maxSensorNumber == 6) {
      x3a = power3;
      x3b = 0;
    }else if (sensorInfo.maxSensorNumber == 7) {
      x3a = power3;
      x3b = 0;
    }else if (sensorInfo.maxSensorNumber == 1) {
     x3a = 0;
      x3b = power3;
    }else if (sensorInfo.maxSensorNumber == 2) {
      x3a = 0;
      x3b = power3;
    }else if (sensorInfo.maxSensorNumber == 3) {
      x3a = power3;
      x3b = 0;
    }
   
   
    
 if (sensorInfo.maxSensorNumber == 0) {
      x4a = 0;
      x4b = power4;
    }
    else if (sensorInfo.maxSensorNumber == 4) {
      x4a = 0;
      x4b = power4;
    }else if (sensorInfo.maxSensorNumber == 5) {
     x4a = power4;
      x4b = 0;
    }else if (sensorInfo.maxSensorNumber == 6) {
      x4a = power4;
      x4b = 0;
    }else if (sensorInfo.maxSensorNumber == 7) {
      x4a = 0;
      x4b = power4;
    }else if (sensorInfo.maxSensorNumber == 1) {
      x4a = power4;
      x4b = 0;
    }else if (sensorInfo.maxSensorNumber == 2) {
      x4a = power4;
      x4b = 0;
    }else if (sensorInfo.maxSensorNumber == 3) {
       x4a = 0;
      x4b = power4;
    }
//モータ出力  
     
     if((event.orientation.x) > 15 && (event.orientation.x) < 180 ){
   siseil();
    
  }else if((event.orientation.x) >= 180 && (event.orientation.x) < 345){
   siseir();
  }
 

     
     if((event.orientation.x) < 15 ) {
      analogWrite(M1a, x1a);
      analogWrite(M1b, x1b);
      analogWrite(M2a, x2a);
      analogWrite(M2b, x2b);
      analogWrite(M3a, x3a);
      analogWrite(M3b, x3b);
      analogWrite(M4a, x4a);
      analogWrite(M4b, x4b);
    SSWsens();
      
     }else if((event.orientation.x) > 345){
      analogWrite(M1a, x1a);
      analogWrite(M1b, x1b);
      analogWrite(M2a, x2a);
      analogWrite(M2b, x2b);
      analogWrite(M3a, x3a);
      analogWrite(M3b, x3b);
      analogWrite(M4a, x4a);
      analogWrite(M4b, x4b);
    SSWsens();
     }
     
    if((event.orientation.x) > 15 && (event.orientation.x) < 180 ){
   siseil();
   
    
  }else if((event.orientation.x) >= 180 && (event.orientation.x) < 345){
   siseir();
  
   
  }
  
}
}

void serialPrintAllPusleWidth(float *pulseWidth, sensorInfo_t *infop) {
    if(state == 0){
    for(int i = 0; i < IR_NUM; i++) {
        Serial.print(pulseWidth[i]); 
        Serial.print("\t");
    }
    Serial.print(infop->activeSensors); 
    Serial.print("\t");
    Serial.print(infop->maxSensorNumber);
    Serial.print("\t");
    Serial.print(infop->maxPulseWidth); 
}
 }

void serialPrintVectorXY(vectorXY_t *self) {
    if(state == 0){
    Serial.print(self->x); 
    Serial.print("\t");
    Serial.print(self->y);
}
}
void serialPrintVectorRT(vectorRT_t *self) {
     if(state == 0){
    Serial.print(self->radius);
    Serial.print("\t");
    Serial.print(self->theta);
}
}
void Stop(void){
    analogWrite(M1a,0);  //モータ全て 反時計回り
    analogWrite(M1b,0);
    analogWrite(M2a,0);
    analogWrite(M2b,0);
    analogWrite(M3a,0);
    analogWrite(M3b,0);
    analogWrite(M4a,0);
    analogWrite(M4b,0);
}
void siseir(void){
    analogWrite(M1a,43);  //モータ全て 反時計回り
    analogWrite(M1b,0);
    analogWrite(M2a,0);
    analogWrite(M2b,43);
    analogWrite(M3a,43);
    analogWrite(M3b,0);
    analogWrite(M4a,43);
    analogWrite(M4b,0);
    
    
}
void siseil(void){
    analogWrite(M1a,0);  //モータ全て 反時計回り
    analogWrite(M1b,43);
    analogWrite(M2a,43);
    analogWrite(M2b,0);
    analogWrite(M3a,0);
    analogWrite(M3b,43);
    analogWrite(M4a,0);
    analogWrite(M4b,43);
    
}
void SSWsens(void){
  ssw1();
ssw2();
  if(Distance1 <= 25){
  analogWrite(2,0);
  analogWrite(3,200);
 
  analogWrite(4,0);
  analogWrite(5,200);
  
  analogWrite(6,0);
  analogWrite(7,200);
 
  analogWrite(8,200);
  analogWrite(9,0);
 digitalWrite(buzzer,HIGH);
 delay(50);
 digitalWrite(buzzer,LOW);
 delay(50);
  }else if(Distance2 <= 25 ){
  analogWrite(2,200);
  analogWrite(3,0);

  analogWrite(4,200);
  analogWrite(5,0);
  
  analogWrite(6,200);
  analogWrite(7,0);
 
  analogWrite(8,0);
  analogWrite(9,200);
  digitalWrite(buzzer,HIGH);
 delay(50);
 digitalWrite(buzzer,LOW);
 delay(50);
  }
}
  void INA(void){
float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
 
  
if(busvoltage <= 7){ 
digitalWrite(buzzer,HIGH);
 delay(50);
 digitalWrite(buzzer,LOW);
 delay(50);
 }
 }
