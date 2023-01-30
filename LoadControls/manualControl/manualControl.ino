
#include<Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 adc;  // Construct an ads1115 
Adafruit_MCP4725 dac;


float voltageA;
float voltageB;

float currentA;
float currentB;

float voltageScaling = 15.78; 
float currentVOffset = 0.0917;
float currentVscaling = 0.9362; 

void readParameters() {

//Read Values off of the Teensy 
int voltageBitTeensy = analogRead(A0);
int currentBitTeensy = analogRead(A1);
//Convert 10Bit to V & I 
voltageA = ((voltageBitTeensy/1023.00)*3.30*voltageScaling); 
currentA = ((((currentBitTeensy/1023.00)*3.30)-currentVOffset)/currentVscaling); 



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
