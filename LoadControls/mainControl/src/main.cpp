#include <Arduino.h>
#include<Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>


Adafruit_ADS1115 adc1;  // Construct an ads1115 
Adafruit_MCP4725 dac1;

//Globals
float vLocal, vADC; 
float iLocal, iADC; 
float resLocal, resADC;

typedef struct{
  float v, i, r; 
} ioRead; 

ioRead teensyIO; 
ioRead adcIO; 
//global constants; 
float voltageScaling = 15.78; 
float currentVOffset = 0.0917;
float currentVscaling = 0.9362; 

float errMax = 1.0; 
float errMin = -errMax; 


//IO Functions 
ioRead readLocalParams(){
  //ReadTeesnyAnalogPins
  int voltageBitTeensy = analogRead(A0);
  int currentBitTeensy = analogRead(A1);
  //Convert 10Bit to V & I 
  vLocal = ((voltageBitTeensy/1023.00)*3.30*voltageScaling); 
  iLocal = ((((currentBitTeensy/1023.00)*3.30)-currentVOffset)/currentVscaling);
  resLocal  = vLocal/iLocal; 
  teensyIO = {vLocal,iLocal,resLocal};
  Serial.print(teensyIO.v);
  Serial.print(","); 
 
  Serial.print(teensyIO.i);
  Serial.print(teensyIO.r); 
  return teensyIO; 
}

ioRead readADCParams(){
  //ReadADCAnalogPins
  int voltageBitADC = adc1.readADC_SingleEnded(0);
  int currentBitADC = adc1.readADC_SingleEnded(1);
  //Convert 16Bit to V & I 
  vADC = ((voltageBitADC/65536.00)*5.00*voltageScaling); 
  iADC = ((((currentBitADC/65536.00)*5.00)-currentVOffset)/currentVscaling);
  resADC  = vADC/iADC; 
  adcIO = {vADC,iADC,resADC};
  //Serial.println(adcIO);
  return adcIO; 
}

//Control State Functions
float constR(float R, float V, float I, float setPoint){
  float currentTime = millis(); //counting time
  float dt = (float)(currentTime - previousTime)/1000.0; //time increment dt
  float err = setPoint- R; //computing the error
  err = constrain(err, errMin, errMax); //anti-integral windup 

  //Proportional
  float comPI = Kpi*err; //proportional component

  //Derivative
  float errDer = (err-errPrev)/dt; //Derivative of erro
  float comD = Kd*errDer; //derivative component
  errPrev = err; //resetting error
  
  //sum of all command components
  float comStroke = comPI + comD; //+ comI;

  stroke_percent = stroke_curr + comStroke*dt; //first option
  
  stroke_curr = stroke_percent; //resetting current stroke
  float previousTime = currentTime; //resetting time
}






void setup() {
  // put your setup code here, to run once:
  adc1.begin();
  dac1.begin(0x62);
}

void loop() {
  // put your main code here, to run repeatedly:

}