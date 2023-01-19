//LOAD SIDE ARDUINO
//For steady-state conditions, we want the relay to be de-energized.
//For braking conditions, sensor will read low voltage and low current below an acceptable threshold
//Once we leave braking conditions, 

//global variable declarations
float nextprint;
unsigned long previousTime = 0;
unsigned const interval = 1;
const int n = 50; //Number of samples 
const int relayPin = 9; //pin that switches NO relay to close

//pin declarations
int voltPin = A0;
int currPin = A1;

//sensor values
float readVoltage; //ADC conversion
float readCurrent; //analog reading from current sensor
float voltSense; //analog reading from voltage divider
float currSense; //analog reading from current
float avgVoltage; //average voltage reading
float avgCurrent;  //average current reading
float measPower; // average voltage times average current
float currOffset = 0.004848368; //determined from calibration (load side INA 169)
float voltOffset = 0.333946348; //determined from calibration (R1 = 1kOhm, R2 = 14.7kOhm)

//control law variables
float lowVoltThresh = 6.0; //low voltage threshhold (V)
float lowCurrThresh = 0.15; //low current threshhold (A)
unsigned long currentTime; //for delay
unsigned long stateDelay = 10000; // delay for state 3

; //low current threshold (A)
float cutInPower = 1.3; //cut in power (W)
int state = 1; //initial state is cut-in

//for printing
unsigned long nextPrint = 1000;

void setup() {
  pinMode(relayPin, OUTPUT);
  Serial.begin(9600);
}

//Reading the sensor
void readParameters() {

 //compute voltage (V)
voltSense = analogRead(voltPin);
readVoltage = 16.0*voltSense*(5.0/1023.0); //different math for teensy!!!
if (readVoltage < 0) {
  readVoltage = 0;
}
 
 //compute current (A)
if (readVoltage >= 2.3) {
currSense = analogRead(currPin); //1 V for 1 A
readCurrent = (currSense*5.0/1023.0) + currOffset;
}

else {
  readCurrent = 0;
}

}

void avgPowerCalculator() {

  //initialize 1st variables
  float sumVoltage = 0;
  float sumCurrent = 0;

  //compute average and call that the power measured
  for (int i = 1; i <= n; i = i + 1) {
    readParameters();
    sumVoltage = readVoltage + sumVoltage; //adding up voltages
    sumCurrent = readCurrent + sumCurrent; //adding up currents

    if (i == n) {
      avgVoltage = sumVoltage/(float)n; //averaging voltage
      avgCurrent = sumCurrent/(float)n; //averaging current
      measPower = avgVoltage*avgCurrent;
      }
    }
  }

//sensor debugging
void printVal() {

if (millis() > nextprint) {

Serial.print("State is: ");
Serial.print(state);
Serial.print(", Relay state is: ");
Serial.print(digitalRead(relayPin));
Serial.print(", Power is: ");
Serial.print(measPower);
Serial.print(", Voltage is: ");
Serial.print(avgVoltage);
Serial.print(", Current is: ");
Serial.println(avgCurrent);

nextprint = millis()+1.0;
  }
}

void loop() {
  avgPowerCalculator(); //collect data
  printVal();
  
  switch (state) {
    case 1: //Start up conditions  
    digitalWrite(relayPin, LOW);
    if (measPower >= cutInPower) {
    state = 2; //into power production, where braking is possible
      break;
    }
    break;

    case 2: //normal operations
    
    digitalWrite(relayPin, LOW);
    if ((avgVoltage < lowVoltThresh) && (avgCurrent < lowCurrThresh)) {
      digitalWrite(relayPin, HIGH); //Switch to closed battery circuit
      currentTime = millis(); //initialize time for delay
      state = 3; //moving towards brake: close battery, open load
      break; 
    }
    break;

  case 3: 
   
    if ((avgCurrent > 0.05) && (millis() > (currentTime + stateDelay))) {
      state = 1; //out of failure mode: close load, open battery
      break;
    }
    break;
  }

}
