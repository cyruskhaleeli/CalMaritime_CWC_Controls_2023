
#include<Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 adc;  // Construct an ads1115 
Adafruit_MCP4725 dac;

int vPin=14; 
int iPin=15;
unsigned long previousTime = 0; //initialize code to zero
const int n = 50; //Number of samples 
unsigned long nextprint; 


int voltageBit1;
int currentBit1;
int voltageBit2;
int currentBit2;

float realVoltage1;
float realCurrent1;
float realVoltage2;
float realCurrent2;

float teensyResistance; 
float i2cResistance; 

float voltageScaling = 15.78; 
float currentVOffset = 0.0917;
float currentVscaling = 0.9362; 



void readTeensy(){
  
voltageBit1 = analogRead(vPin);
realVoltage1 = (voltageBit1/1023)*3.3*voltageScaling; 
Serial.print("Teesny Measured V:");
Serial.println(realVoltage1);

currentBit1 = analogRead(iPin);
realCurrent1 = (((currentBit1/1023)*3.3)-currentVOffset); 
Serial.print("Teensy Measured I:");
Serial.println(realCurrent1);

teensyResistance = realCurrent1/realVoltage1; 
Serial.print("Teensy Measured Resistance:");
Serial.println(teensyResistance);
}

void readADC(){
  voltageBit2 = adc.readADC_SingleEnded(0);
  currentBit2 = adc.readADC_SingleEnded(1);
 
  
  realVoltage2 = (voltageBit1/65535)*5*voltageScaling; 
  Serial.print("I2C Measured V:");
  Serial.println(realVoltage1);
  
  realCurrent2 = (((currentBit2/65535)*5)-currentVOffset); 
  Serial.print("I2C Measured I:");
  Serial.println(realCurrent2);
  
  i2cResistance = realCurrent2/realVoltage2; 
  Serial.print("I2C Measured Resistance:");
  Serial.println(i2cResistance);

}
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  dac.begin(0x62);
  adc.begin();
  Serial.println("Initialized");
}

void loop() {
  //readADC();
  readTeensy();
   /*int bitValue = Serial.parseInt();
   int sensorValue = analogRead(A1);
     if (Serial.available() > 0) {
            Serial.print("New Voltage Out:");
            Serial.println(bitValue);
            dac.setVoltage(bitValue,false); 
            delay(2000);
          }
    Serial.println(sensorValue);*/

}
