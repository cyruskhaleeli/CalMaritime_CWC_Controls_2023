#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>
#include <SD.h>
#include <SPI.h>


// Pin Define 

#define adcVoltagePin 0
#define adcCurrentPin 1
#define teensyVoltagePin A0
#define teensyCurrentPin A1 



//Create I2C Device Objects 
Adafruit_ADS1115 adc1;  // Construct an ads1115 
Adafruit_MCP4725 dac1; // Construct an MCP4725

//General Clock Globals 
unsigned long clock = millis();
unsigned long previousTime = 0; 
//IO Globals
float vLocal, vADC; //Voltage by Source 
float iLocal, iADC;  //Current by Source
float resLocal, resADC; //Resistance by source 
float voltageScaling = 15.78;  
float currentVOffset = 0.0917;
float currentVscaling = 0.9362; 

//Type Defining IORead as a Struct
typedef struct{
  float v, i, r; 
} ioRead; 

ioRead teensyIO; 
ioRead adcIO; 

//IO Functions 
ioRead readLocalParams(){
  //ReadTeesnyAnalogPins
  int voltageBitTeensy = analogRead(teensyVoltagePin);
  int currentBitTeensy = analogRead(teensyCurrentPin);
  //Convert 10Bit to V & I 
  vLocal = (float)((voltageBitTeensy/1023.00)*3.30*voltageScaling); 
  iLocal = (float)((((currentBitTeensy/1023.00)*3.30)-currentVOffset)/currentVscaling);
  resLocal  = vLocal/iLocal; 
  teensyIO = {vLocal,iLocal,resLocal};
  Serial.print(teensyIO.v);
  Serial.print(","); 
  Serial.print(teensyIO.i);
  Serial.print(","); 
  Serial.println(teensyIO.r); 
  return teensyIO; 
}

ioRead readADCParams(){
  //ReadADCAnalogPins
  int voltageBitADC = adc1.readADC_SingleEnded(adcVoltagePin);
  int currentBitADC = adc1.readADC_SingleEnded(adcCurrentPin); 
  //Convert 16Bit to V & I 
  vADC = (float)((voltageBitADC/65536.00)*5.00*voltageScaling); 
  iADC = (float)((((currentBitADC/65536.00)*5.00)-currentVOffset)/currentVscaling);
  resADC  = vADC/iADC; 
  adcIO = {vADC,iADC,resADC};
  //Serial.println(adcIO);
  return adcIO; 
}
//Control Laws

///Globals
float setPoint = 0;
float err;
float errPrev = 0;
float errSum = 0l;
int state = 0; 
float modePrevious; 
int dacOutput = 0; 

typedef struct { 
  float Kp,Kd,Ki,errMax,errMin;
} ctrlValues; 

  ctrlValues constPWR_Values = {0,0,0,10,-10};
  ctrlValues constRES_Values = {0,0,0,10,-10};
  ctrlValues constVOL_Values = {0,0,0,2,-2};
  ctrlValues constCUR_Values = {0,0,0,0.2,-0.2};



void ctrlLaw(float sP,float mV,ctrlValues value,int state_val) {
  if ( state_val != modePrevious)
  {
   err = 0; 
   errPrev = 0; 
   errSum = -0;
   modePrevious = state_val; 
  }
  
  float dt = (float)(clock-previousTime)/1000.00; //time increment dt
  err = sP - mV;
  err = constrain(err,value.errMax,value.errMin); 

  //Porportional Control
  float P = value.Kp*err; 

  //Derivative Control 
  float errDer = (err-errPrev)/dt; 
  float D = value.Kd*errDer; 
  errPrev = err;

  //Integral Control
  errSum += err; 
  float I = value.Ki*errSum; 
  //Control Output
  dacOutput = (int)(P+I+D)*(4096/5); 
  dacOutput = constrain(dacOutput, 0, 4096); 
  dac1.setVoltage(dacOutput, false);
}
/*float constRes(){

}
float constV(){

}
float constI(){

}

*/







void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  adc1.begin();
  dac1.begin(0x62);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (state)
  {
  case 0 /* constant-expression */:
    
    break;
  
  default:
    break;
  }




}
