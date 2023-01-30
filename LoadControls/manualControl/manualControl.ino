
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


float voltageA;
float voltageB;

float currentA;
float currentB;

float voltageScaling = 15.78; 
float currentVOffset = 0.0917;
float currentVscaling = 0.9362; 


//Read Values off of the Teensy 
int voltageBitTeensy = analogRead(A0);
int currentBitTeensy = analogRead(A1);
//Convert 10Bit to V & I 
voltageA = ((voltageBitTeensy/1023.00)*3.30*voltageScaling); 
currentA = ((((currentBitTeensy/1023.00)*3.30)-currentVOffset)/currentVscaling); 

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


//ReadValues off of the ADC
//int voltageBitADC = adc.readADC_SingleEnded(0);
//int currentBitADC = adc.readADCSingleEnded(1);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  dac.begin(0x62);
  adc.begin();
  Serial.println("Initialized");
}

float resistance(float v1, float i1){
  float resistance; 
  resistance = v1/i1; 
  return resistance; 
}

void loop() {
   
}
