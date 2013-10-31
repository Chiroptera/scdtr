// libraries for the timer interrupt
#include <avr/io.h>
#include <avr/interrupt.h>


/**********************************************************
 **********************************************************
 **********************************************************
 *****
 *****                     VARIABES 
 ***** 
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
 * LDR
 **********************************************************/
const int LDRpin = A0; //input pin for the LDR sensor

const int ledLightLow =8; //pin for LED for light off
const int ledLightHigh =7; //pin for LED for light on

const int VoltDiv =10000; //resistance used with the LDR for the voltage divisor
const int LDRvcc =5; //voltage used to supply the voltage divisor
const int LDR_uplimit = 13000;
const int LDR_downlimit = 1000;

float LDRValue = 0; // variable to store the value coming from the sensor
float LDRVolt =0; // LDR tension
float LDRes=0; // LDR resistance

//moving average variables
const int ldrNumReadings=20; // number of readings to compute moving average
float ldrReadings[ldrNumReadings]; // the readings from the analog input
int ldrIndex = 0; // the index of the current reading
float ldrTotal = 0; // the running total
float ldrAverage = 0; // the average

/**********************************************************
 * PROXIMITY
 **********************************************************/
const int proxSensor = A5; //input pin fo the proximity sensor
const int LedPresence = 3; // select the pin for the LED that detects presence

float proximity, distance;

// moving average variables
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

float realtemp, temperature;

// moving average variables
const int tempNumReadings=5; // number of readings to compute moving average
float tempReadings[tempNumReadings]; // the readings from the analog input
int tempIndex = 0; // the index of the current reading
float tempTotal = 0; // the running total
float tempAverage = 0; // the average


/**********************************************************
 *                   FAN 
 **********************************************************/
const int fanPin = 5;
const int potentiometerFanPin = A1;

unsigned long pidFan_lastTime; //for constant sample time assurance
int pidFan_sampleTime = 10;
double pidFan_y, pidFan_u; //input reading and output value
double pidFan_ref=31; //reference
double pidFan_errorSum, pidFan_lastError; //error sum and previous erros for integral and derivative components
double pidFan_kp=22, pidFan_ki=0.01, pidFan_kd=0; //proportional, integral and derivative variables

/**********************************************************
 *                   LUMINAIRE
 **********************************************************/
const int ledLight = 6;
const int potentiometerLEDPin = A2;

double pidLED_y, pidLED_u; //input reading and output value
double pidLED_ref=100000; //reference for the light resistance
double pidLED_errorSum, pidLED_lastError; //error sum and previous erros for integral and derivative components
double pidLED_kp=0.005, pidLED_ki=0, pidLED_kd=0; //proportional, integral and derivative variables


/****************************************************
 *
 *                 PID control
 *
 *****************************************************/

ISR(TIMER1_COMPA_vect){  //interrupt code

  /*****************************
   *             LED
   ******************************/
  // get latest reading
  pidLED_y=ldrAverage;

  double error = pidLED_ref - pidLED_y; //compute current error
  pidLED_errorSum += (error * pidLED_sampleTime); //error's integral
  int errorDerivative = (error - pidLED_lastError) / 10; //error's derivative

  // calculation of PID ouput
  pidLED_u = pidLED_kp * error + pidLED_ki * pidLED_errorSum + pidLED_kd * errorDerivative;

  // save error for future derivative
  pidLED_lastError=error;

  // if PID output exceeds 255, then it is 255; if it is negative, then it is 0
  if (pidLED_u > 255) pidLED_u=255;
  if (pidLED_u < 0) pidLED_u=0;   

  /*****************************
   *             FAN
   ******************************/

  //get latest reading
  pidFan_y=tempAverage;

  error =  pidFan_y - pidFan_ref; //compute current error
  pidFan_errorSum += (error * pidFan_sampleTime); //error's integral
  errorDerivative = (error - pidFan_lastError) / 10; //error's derivative

  //anti windup for the fan
  if(pidFan_ki*pidFan_errorSum > 255) pidFan_errorSum = 255/pidFan_ki;
  if(pidFan_ki*pidFan_errorSum < -255) pidFan_errorSum = -255/pidFan_ki;

  // calculation of PID ouput
  pidFan_u = pidFan_kp * error + pidFan_ki * pidFan_errorSum + pidFan_kd * errorDerivative;

  // save error for future derivative
  pidFan_lastError=error;

  if(pidFan_u > 150) pidFan_u = 150; //150 is the max of fan for noiseless performance. Restricts control within the boundary.

  //if(pidFan_u<0) return;  // se a luz ainda no aqueceu, deixa aquecer

  if(error<0) pidFan_u=0;  // if temperature hasn't exceeded setpoint, don't act

}


void setup() {

  cli(); //disable all interrupts

  //proximity
  //pinMode(proxSensor, INPUT);
  pinMode(LedPresence, OUTPUT);


  //temperature
  //pinMode(tempSensor, INPUT);
  pinMode(LedTemperature, OUTPUT);

  // light sensor
  pinMode(ledLightLow, OUTPUT);
  pinMode(ledLightHigh, OUTPUT);

  // luminaire
  //pinMode(potentiometerLEDPin, INPUT);
  pinMode(ledLight, OUTPUT);

  // fan
  //pinMode(potentiometerFanPin, INPUT);
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

  ////Set interrupt for the PID timer
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 625;// = (16*10^6) / (100*256) - 1 -> we want a frequency of 100 Hz (10ms) using a prescale of 256
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); //enable all interrupts

}




void loop()
{

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
  if ( (realtemp > 1.2 * tempAverage || realtemp < 0.8 * tempAverage) && millis() > 5000) realtemp=tempAverage;

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

  //    Serial.println("\t read"); 
  //    Serial.print(realtemp);
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

  //convert the LDR reading to a voltage
  LDRVolt = (5.00*LDRValue)/1023.00;

  //compute LDR resistance
  //LDRes= ( LDRvcc*VoltDiv- LDRVolt*VoltDiv) / LDRVolt; //old formula
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


  //Serial.print("Analog reading[LDR]:");
  //Serial.println(LDRValue);
  //Serial.print("Voltage [LDR]:");
  //Serial.println(LDRVolt);
  //Serial.print("Resistance [LDR]:");
  //Serial.println(LDRes);
  //Serial.println("--");

  /**********************************************************
   *
   * DRIVING THE FAN
   *
   **********************************************************/

  double potentiometerFanValue = analogRead(potentiometerFanPin);
  potentiometerFanValue=map(potentiometerFanValue,0,1023,0,1);

  // writes latest PID value to fan
  analogWrite(fanPin,pidFan_u);


  //Serial.print("Pot fan:"); 
  //Serial.println(potentiometerFanValue,DEC);


  /**********************************************************
   *
   * DRIVING THE LED
   *
   **********************************************************/

  // writes latest PID value to luminaire
  analogWrite(ledPin,pidLED_u);

  int potentiometerLEDValue = analogRead(potentiometerLEDPin);
  potentiometerLEDValue=map(potentiometerLEDValue,0,1023,0,255);


  //Serial.print("Pot luminaire:");
  //Serial.println(potentiometerLEDValue,DEC);


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


