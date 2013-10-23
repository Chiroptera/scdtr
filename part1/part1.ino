/*********************************************************
**********************************************************
***** *****
***** VARIABES *****
***** *****
**********************************************************
**********************************************************
*********************************************************/

const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;

/**********************************************************
* OPERATION MODES
**********************************************************/
const int pushbutton1 = 12;
const int pushbutton2 = 13;

/**********************************************************
* LUMINAIRE
**********************************************************/
const int ledLight = 6;
const int potentiometerLEDPin = A2;
int luminaireValue;

const int lumiNumReadings=20; // number of readings to compute moving average
int lumiReadings[lumiNumReadings]; // the readings from the analog input
int lumiIndex = 0; // the index of the current reading
int lumiTotal = 0; // the running total
int lumiAverage = 0; // the average

/**********************************************************
* FAN
**********************************************************/

const int fanPin = 5;
const int potentiometerFanPin = A1;

/**********************************************************
* LDR
**********************************************************/
const int LDRpin = A0; //input pin for the LDR sensor

const int ledLightLow =8; //pin for LED for light off
const int ledLightHigh =7; //pin for LED for light on

const int VoltDiv =10000; //resistance used with the LDR for the voltage divisor
const int LDRvcc =5; //voltage used to supply the voltage divisor
const int LDR_uplimit = 25000;
const int LDR_downlimit = 8000;

int LDRValue = 0; // variable to store the value coming from the sensor
float LDRVolt =0; // LDR tension
float LDRes=0; // LDR resistance

const int ldrNumReadings=20; // number of readings to compute moving average
int ldrReadings[ldrNumReadings]; // the readings from the analog input
int ldrIndex = 0; // the index of the current reading
int ldrTotal = 0; // the running total
int ldrAverage = 0; // the average

/**********************************************************
* PROXIMITY
**********************************************************/
const int proxSensor = A5; //input pin fo the proximity sensor
const int LedPresence = 3; // select the pin for the LED that detects presence

const int proximityNumReadings=5; // number of readings to compute moving average
int proximityReadings[proximityNumReadings]; // the readings from the analog input
int proximityIndex = 0; // the index of the current reading
int proximityTotal = 0; // the running total
int proximityAverage = 0; // the average


/**********************************************************
* TEMPERATURE
**********************************************************/
const int tempSensor = A4; //input pin for he temperature sensor
const int LedTemperature = 4; // select the pin for the LED that detects temperatures above 50ºC

const int tempNumReadings=5; // number of readings to compute moving average
int tempReadings[tempNumReadings]; // the readings from the analog input
int tempIndex = 0; // the index of the current reading
int tempTotal = 0; // the running total
int tempAverage = 0; // the average




/**********************************************************
* PID FAN
**********************************************************/

unsigned long pidFan_lastTime; //for constant sample time assurance
int pidFan_sampleTime = 10;
double pidFan_y, pidFan_u; //input reading and output value
double pidFan_ref=25; //reference
double pidFan_errorSum, pidFan_lastError; //error sum and previous erros for integral and derivative components
double pidFan_kp=1, pidFan_ki=0, pidFan_kd=100; //proportional, integral and derivative variables


//    P-Control: P=0.50*Gu, I=0, D=0.
//    PI-Control: P=0.45*Gu, I=1.2/tu, D=0.
//    PIDFAN-Control: P=0.60*Gu, I=2/tu, D=tu/8. 


void pidFan(){
//pidFan_y=analogRead(PIN_Y);
pidFan_y=tempAverage;

unsigned long now = millis();
double timeChange = (double) (now - pidFan_lastTime);
if (timeChange < pidFan_sampleTime) return; //don't compute output if sample time has not been reached
double error = pidFan_ref - pidFan_y; //compute current error
pidFan_errorSum += (error * pidFan_sampleTime); //error's integral
int errorDerivative = (error - pidFan_lastError) / pidFan_sampleTime; //error's derivative
pidFan_u = pidFan_kp * error + pidFan_ki * pidFan_errorSum + pidFan_kd * errorDerivative;
pidFan_lastError=error;
pidFan_lastTime=now;
//map pidFan_u
//analogWrite(PIN_U,pidFan_u)

}


/**********************************************************
* PID LED 
**********************************************************/

unsigned long pidLED_lastTime; //for constant sample time assurance
int pidLED_sampleTime = 10;
double pidLED_y, pidLED_u; //input reading and output value
double pidLED_ref=25000; //reference for the light resistance
double pidLED_errorSum, pidLED_lastError; //error sum and previous erros for integral and derivative components
double pidLED_kp=1, pidLED_ki=0, pidLED_kd=100; //proportional, integral and derivative variables


//    P-Control: P=0.50*Gu, I=0, D=0.
//    PI-Control: P=0.45*Gu, I=1.2/tu, D=0.
//    PIDFAN-Control: P=0.60*Gu, I=2/tu, D=tu/8. 


void pidLED(){

  pidLED_y=ldrAverage;
  
  unsigned long now = millis();
  double timeChange = (double) (now - pidLED_lastTime);
  if (timeChange < pidLED_sampleTime) return; //don't compute output if sample time has not been reached
  
  double error = pidLED_ref - pidLED_y; //compute current error
  pidLED_errorSum += (error * pidLED_sampleTime); //error's integral
  int errorDerivative = (error - pidLED_lastError) / pidLED_sampleTime; //error's derivative
  
  pidLED_u = pidLED_kp * error + pidLED_ki * pidLED_errorSum + pidLED_kd * errorDerivative;
  
  pidLED_lastError=error;
  pidLED_lastTime=now;
  
  
  Serial.print("PID_OUTPUT:\t");
  Serial.println(pidLED_u,DEC);
  Serial.print("LDR RES:\t");
  Serial.println(ldrAverage,DEC);
  
  //map pidLED_u
  //analogWrite(PIN_U,pidLED_u)

}



void setup() {

  //proximity
  pinMode(proxSensor, INPUT);
  pinMode(LedPresence, OUTPUT);

  
  //temperature
  pinMode(tempSensor, INPUT);
  pinMode(LedTemperature, OUTPUT);

  // light sensor
  pinMode(ledLightLow, OUTPUT);
  pinMode(ledLightHigh, OUTPUT);
  
  // luminaire
  pinMode(potentiometerLEDPin, INPUT);
  pinMode(ledLight, OUTPUT);
  
  // fan
  pinMode(potentiometerFanPin, INPUT);
  pinMode(fanPin, OUTPUT);

  //
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  //operation mode
  pinMode(pushbutton1, OUTPUT);
  pinMode(pushbutton2, OUTPUT);
  
  // initialize serial communication:
  Serial.begin(9600);
}



void loop()
{

  //delay(1000);
  int proximity, temperature, distance;
  float realtemp;

  Serial.println(pidLED_kp,DEC);
  Serial.println(pidLED_ki,DEC);
  Serial.println(pidLED_kd,DEC);
  
  if (Serial.available() > 0) { //If we sent the program a command deal with it
    for (int x = 0; x < 4; x++) {
      switch (x) {
        case 0:
          pidLED_kp = Serial.parseFloat();
          break;
        case 1:
          pidLED_ki = Serial.parseFloat();
          
          break;
        case 2:
          pidLED_kd = Serial.parseFloat();
          break;
        case 3:
          for (int y = Serial.available(); y == 0; y--) {
            Serial.read(); //Clear out any residual junk
          }
          break;
      }
    }
  }




/**********************************************************
*
* Proximity
*
**********************************************************/

  // read from proximity sensor
  proximity = analogRead(proxSensor);
  
  //// moving average to reduce noise
  // subtract the last reading:
  proximityTotal= proximityTotal - proximityReadings[proximityIndex];
  
  // read from the sensor:
  proximityReadings[proximityIndex] = proximity;
  
  // add the reading to the total:
  proximityTotal= proximityTotal + proximityReadings[proximityIndex];
  
  // advance to the next position in the array:
  proximityIndex = proximityIndex + 1;

  // if we're at the end of the array...
  if (proximityIndex >= proximityNumReadings)
    // ...wrap around to the beginning:
    proximityIndex = 0;

  // calculate the average:
  proximityAverage = proximityTotal / proximityNumReadings;

  //calculate distance
  distance = -0.146 * proximityAverage + 63.87;
  
  // workstation occupied if presence detected within 50 cm
  if (distance < 50) digitalWrite(LedPresence, HIGH);
  else digitalWrite(LedPresence, LOW);

  // equation y=-0.146x + 63.87 (in cm); it's linear in the interval [20,50] (cm)


/*
Serial.print("proximity:");
Serial.print(proximityAverage);
Serial.print("\t distance:");
Serial.println(distance);
*/


/**********************************************************
*
* Temperature
*
**********************************************************/

  // get temperature reading
  temperature = analogRead(tempSensor);
  
  // calculate real temperature from reading
  realtemp=( ( 5 * (float)temperature ) / 1023 ) / 0.01;
  
  // filter that doesn't accept readings which vary more than 10% from moving average
  if ( (realtemp > 1.1 * tempAverage || realtemp < 0.9 * tempAverage) && millis() > 5000) realtemp=tempAverage;
  
  //// moving average to reduce noise
  // subtract the last reading:
  tempTotal= tempTotal - tempReadings[tempIndex];
  // read from the sensor:
  tempReadings[tempIndex] = realtemp;
  // add the reading to the total:
  tempTotal= tempTotal + tempReadings[tempIndex];
  // advance to the next position in the array:
  tempIndex = tempIndex + 1;

  // if we're at the end of the array...
  if (tempIndex >= tempNumReadings)
    // ...wrap around to the beginning:
    tempIndex = 0;

  // calculate the average:
  tempAverage = tempTotal / tempNumReadings;

  //if temperature reaches 50º, LED ir turned on
  if (tempAverage > 50) digitalWrite(LedTemperature, HIGH);
  else digitalWrite(LedTemperature, LOW);


//  Serial.print("\t Temperature:");
//  Serial.print(tempAverage);
//  Serial.println();




/**********************************************************
*
* LDR
*
**********************************************************/

  // read the value from the sensor
  LDRValue = analogRead(LDRpin);
  
  //Serial.print("Analog reading[LDR]:");
  //Serial.println(LDRValue);

  //convert the LDR reading to a voltage
  LDRVolt = (5.00*LDRValue)/1023.00;
  //LDRVolt = map(sensorValue, 0, 1023, 0.00 , 5.00);
  
  //Serial.print("Voltage [LDR]:");
  //Serial.println(LDRVolt);

  //compute LDR resistance
  //LDRes= ( LDRvcc*VoltDiv- LDRVolt*VoltDiv) / LDRVolt;
    LDRes = ( VoltDiv * LDRVolt ) / ( 5 - LDRVolt);
    
      //// moving average to reduce noise
  // subtract the last reading:
  ldrTotal= ldrTotal - ldrReadings[ldrIndex];
  
  // read from the sensor:
  ldrReadings[ldrIndex] = LDRes;
  
  // add the reading to the total:
  ldrTotal= ldrTotal + ldrReadings[ldrIndex];
  
  // advance to the next position in the array:
  ldrIndex = ldrIndex + 1;

  // if we're at the end of the array...
  if (ldrIndex >= ldrNumReadings)
    // ...wrap around to the beginning:
    ldrIndex = 0;

  // calculate the average:
  ldrAverage = ldrTotal / ldrNumReadings;

  //Serial.print("Resistance [LDR]:");
  //Serial.println(LDRes);
  //Serial.println("--");

  //change state LED
  if (LDRes < LDR_downlimit){ //there is enough light in the room
    digitalWrite(ledLightLow, HIGH);
    digitalWrite(ledLightHigh, LOW);
  }
  else if(LDRes > LDR_uplimit){ //there is not enough light in the room
    digitalWrite(ledLightHigh, HIGH);
    digitalWrite(ledLightLow, LOW);
  }
  else{ //there is so-so light in the room
    digitalWrite(ledLightLow, LOW);
    digitalWrite(ledLightHigh, LOW);
  }



/**********************************************************
*
* DRIVING THE FAN
*
**********************************************************/

  pidFan();

  int potentiometerFanValue = analogRead(potentiometerFanPin);
  potentiometerFanValue=map(potentiometerFanValue,0,1023,0,255);
  analogWrite(fanPin,pidFan_u);
  
 //Serial.print("Pot fan:");
 
 //Serial.println(pid_u,DEC);
  
  
/**********************************************************
*
* DRIVING THE LED
*
**********************************************************/
  pidLED();
  
  int potentiometerLEDValue = analogRead(potentiometerLEDPin);
  luminaireValue=map(potentiometerLEDValue,0,1023,0,255);
  analogWrite(ledLight,lumiAverage);
  
  //// moving average to reduce noise
  // subtract the last reading:
  lumiTotal= lumiTotal - lumiReadings[lumiIndex];
  
  // read from the sensor:
  lumiReadings[lumiIndex] = luminaireValue;
  
  // add the reading to the total:
  lumiTotal= lumiTotal + lumiReadings[lumiIndex];
  
  // advance to the next position in the array:
  lumiIndex = lumiIndex + 1;

  // if we're at the end of the array...
  if (lumiIndex >= lumiNumReadings)
    // ...wrap around to the beginning:
    lumiIndex = 0;

  // calculate the average:
  lumiAverage = lumiTotal / lumiNumReadings;
  
 Serial.print("Pot luminaire:");
 Serial.println(lumiAverage,DEC);
 //Serial.println(luminaireValue,DEC);

  ///////////////////////////////////////////////////////////
  //Sending data to PC
  /*
int LDRestemp = map(LDRes, 1000000000, 1000, 0, 99);
String LL = (LDRestemp < 10) ? "0" + String((int)LDRestemp,DEC) : String((int)LDRestemp,DEC);
String PP = (distance < 10) ? "0" + String((int)distance, DEC) : String((int)distance, DEC);
String TT = (realtemp < 10) ? "0" + String((int)realtemp, DEC) : String((int)realtemp, DEC);
String CC = (0 < 16) ? "0" + String(0, HEX) : String(0, HEX); //TODO add real value
String DD = (0 < 16) ? "0" + String(0, HEX) : String(0, HEX); //TODO add real value
String messageToPC = LL + PP + TT + CC + DD;
Serial.print(messageToPC);
Serial.print("\n");
*/

/**********************************************************
*
* COMMUNICATION
*
**********************************************************/

  // send data only when you receive data:
  if (Serial.available() > 0) {

    // read the incoming byte:

    int incomingByte1 = Serial.read();
    if(incomingByte1 > 47 && incomingByte1 < 58)
      incomingByte1 = incomingByte1-48;
    else
      incomingByte1 = 10 + (incomingByte1 - 65); //Ascii A is 65


      int incomingByte2 = Serial.read();
    if(incomingByte2 > 47 && incomingByte2 < 58)
      incomingByte2 = incomingByte2-48;
    else
      incomingByte2 = 10 + (incomingByte2 - 65); //Ascii A is 65

    /*
Serial.print("I received: ");
Serial.println(incomingByte1 * 16 + incomingByte2);
*/

    //calculate % value of the control command
    int control = map(incomingByte1 *16 + incomingByte2, 0, 255, 0 , 100);
    //Serial.println(control);
  }


/**********************************************************
*
* OPERATION MODES
*
**********************************************************/

  
  if ( digitalRead(pushbutton1) == HIGH)
      digitalWrite(bluePin, HIGH);
  else
      digitalWrite(bluePin, LOW);

  //delay(100);

}
