  #include <Arduino.h>
  #include <Wire.h>
  //#include <Adafruit_MCP4725.h>
  #include <Adafruit_ADS1X15.h>

//Create I2C Device Objects 
Adafruit_ADS1115 adc1;  // Construct an ads1115 

#define adcWindPin 0 //ADC DP Sensor Measurement Pin
#define adcPotPin 2 //ADC Potentiometer Measurement Pin
void readADCParams(){
  //ReadADCAnalogPins
  int WindBitADC = adc1.readADC_SingleEnded(adcWindPin);
  int PotBitADC = adc1.readADC_SingleEnded(adcPotPin); 
  float _WindADC = adc1.computeVolts(WindBitADC);
  float _PotADC = adc1.computeVolts(PotBitADC);
  Serial.print("Wind Sensor Voltage: "); Serial.println(_WindADC);
//  Serial.print("Wind Sensor BITS: "); Serial.println(WindBitADC);
  Serial.print("Potentiometer Voltage: "); Serial.println(_PotADC);
}

void setup() {
  Serial.begin(9600); 
  adc1.begin();
}

void loop() {
  readADCParams();
float _test = analogRead(A0);
float test = _test*(5.0/1023.0);
//Serial.print("test voltage: "); Serial.println(test); 
//Serial.print("test BITS: "); Serial.println(_test); 

  
  
}
