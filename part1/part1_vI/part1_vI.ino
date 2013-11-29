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


/**********************************************************
 * OPERATION MODES
 **********************************************************/
const int pushbuttonPin = 12;

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

const int redPin = 10;
const int greenPin = 11;
const int bluePin = 9;


/**********************************************************
 * LDR
 **********************************************************/
const int LDRpin = A0; //input pin for the LDR sensor

const int ledLightLow =8; //pin for LED for light off
const int ledLightHigh =7; //pin for LED for light on

const int VoltDiv =10000; //resistance used with the LDR for the voltage divisor
const int LDRvcc =5; //voltage used to supply the voltage divisor
const int LDR_uplimit = 19000;
const int LDR_downlimit = 1500;

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
const int LedPresence = 2; // select the pin for the LED that detects presence

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
 *                 Communication
 **********************************************************/
boolean printFlag=0;
String LR; //Light resistance
String PP; //proximity
String TT; //temperature
String FF; //duty cycle of fan
String LU; //duty cycle of luminaire
String messageToPC;

/**********************************************************
 *                   FAN 
 **********************************************************/
const int fanPin = 3;
const int potentiometerFanPin = A1;


double pidFan_y, pidFan_u; //input reading and output value
double pidFan_ref=33; //reference
double pidFan_errorSum, pidFan_lastError; //error sum and previous erros for integral and derivative components
double pidFan_kp=22, pidFan_ki=0.01, pidFan_kd=0; //proportional, integral and derivative variables

/**********************************************************
 *                   LUMINAIRE
 **********************************************************/
const int ledLight = 6;
const int potentiometerLEDPin = A2;

int outputLEDValue=0;           /* variable that stores the output luminaire value */
double pidLED_y, pidLED_u; //input reading and output value
double pidLED_ref; //reference for the light resistance
double pidLED_errorSum, pidLED_lastError; //error sum and previous erros for integral and derivative components
double pidLED_kp=0.0005, pidLED_ki=0.000005, pidLED_kd=0.02; //proportional, integral and derivative variables


/****************************************************
 *
 *                 PID control
 *
 *****************************************************/

ISR(TIMER1_COMPA_vect){  //interrupt code

    printFlag=1;

  /*****************************
   *             LED
   ******************************/
  // get latest reading
  pidLED_y=ldrAverage;

  double error = pidLED_ref - pidLED_y; //compute current error
  pidLED_errorSum += (error * 10); //error's integral
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
  pidFan_errorSum += (error * 10); //error's integral
  errorDerivative = (error - pidFan_lastError) / 10; //error's derivative

  //anti windup for the fan
  if(pidFan_ki*pidFan_errorSum > 255) pidFan_errorSum = 255/pidFan_ki;
  if(pidFan_ki*pidFan_errorSum < -255) pidFan_errorSum = -255/pidFan_ki;

  // calculation of PID ouput
  pidFan_u = pidFan_kp * error + pidFan_ki * pidFan_errorSum + pidFan_kd * errorDerivative;

  // save error for future derivative
  pidFan_lastError=error;

  pidFan_u = (pidFan_u < 0) ? -pidFan_u : pidFan_u;

  if(pidFan_u > 250) pidFan_u = 250;

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

  //operation mode
  pinMode(pushbuttonPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // initialize serial communication:
  Serial.begin(115200);

  ////Set interrupt for the PID timer
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 625;// = (16*10^6) / (100*256) - 1 -> we want a frequency of 100 Hz (10ms) using a prescale of 256
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 256 prescaler
  TCCR1B |= (1 << CS12);// | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);


  // change frequency of pin 3 to reduce fan noise
  const byte mask= B11111000;
  // mask bits that are not prescale !
  int prescale = 1;
  
  TCCR2B = (TCCR2B & mask) | prescale;

  sei(); //enable all interrupts

}

void loop()
{
    //change PID LED reference points
    if (distance < 50) pidLED_ref=18000;
    else pidLED_ref=2000;
  
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
  //Serial.println(distance,DEC);

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

  /**********************************************************
   *
   *                 COMMUNICATION TO PC
   *
   **********************************************************/
   
   LR = String(map(ldrAverage,0,1000000,0,99),DEC);
   if(LR.length() == 1) LR = String("0" + LR);
   
   distance = (distance > 0) ? distance : -distance;
   PP = String((int)distance,DEC);
   if(PP.length() == 1) PP = String("0" + PP);
      
   TT = String((int)((tempAverage > 99) ? 99 : tempAverage),DEC);
   if(TT.length() == 1) TT = String("0" + TT);
      
   FF = String((int)pidFan_u, HEX);
   if (FF.length() == 1) FF = String("0" + FF);
      
   LU = String((int)outputLEDValue, HEX);
   if (LU.length() == 1) LU = String("0" + LU);

   messageToPC = String(LR + PP + TT + FF + LU);

    
   // printFlag turns 1 with 10ms timer
   if (printFlag == 1 && messageToPC.length()==10){
       Serial.println(messageToPC);

       /* Serial.println("LR="+LR); */
       /* Serial.println("PP="+PP); */
       /* Serial.println("TT="+TT); */
       /* Serial.println("FF="+FF); */
       /* Serial.println("LU="+LU); */

       printFlag=0;
   }

  /**********************************************************
   *
   * OPERATION MODES
   *
   **********************************************************/

   


   // read the pushbutton input pin:
   buttonState = digitalRead(pushbuttonPin);

   // compare the buttonState to its previous state
   if (buttonState == HIGH && lastButtonState == LOW) {
       buttonPushCounter++;
       if(buttonPushCounter > 3) buttonPushCounter = 0;
       //Serial.println(buttonPushCounter,DEC);
   }
  
   // save the current state as the last state,
   //for next time through the loop
   lastButtonState = buttonState;

   switch (buttonPushCounter){
   case 1: //Manual
       {
           digitalWrite(redPin, LOW);           
           digitalWrite(greenPin, HIGH);
           digitalWrite(bluePin, LOW);

           //read fan pot and drive fan
           int potentiometerFanValue = analogRead(potentiometerFanPin);
           potentiometerFanValue=map(potentiometerFanValue,0,1023,0,255);
           analogWrite(fanPin,potentiometerFanValue);
      
           //read potentiometer and drive luminaire accordingly
           int potentiometerLEDValue = analogRead(potentiometerLEDPin);
           potentiometerLEDValue = map(potentiometerLEDValue,0,1023,0,255);
           /* analogWrite(ledLight,potentiometerLEDValue);  */
           outputLEDValue=potentiometerLEDValue;
           break;
       }
   case 2: //Serial
       {
           digitalWrite(redPin, LOW);
           digitalWrite(greenPin, LOW);
           digitalWrite(bluePin, HIGH);
       
           // make sure there is data in serial line. In case there is not, keeps the luminaire as it is
           if (Serial.available() == 2) {
               //read the incoming bytes:
               unsigned int incomingByte1 = Serial.read();

               /* 32 is the offset between upper and lower case */
               incomingByte1 = (incomingByte1 > 97) ? incomingByte1 - 32 : incomingByte1;

               if(incomingByte1 > 47 && incomingByte1 < 58) //number
                   incomingByte1 = incomingByte1-48;
               else //letter
                   incomingByte1 = 10 + (incomingByte1 - 65); //Ascii A is 65
    
               unsigned int incomingByte2 = Serial.read();
               incomingByte2 = (incomingByte2 > 97) ? incomingByte2 - 32 : incomingByte2;

               if(incomingByte2 > 47 && incomingByte2 < 58)
                   incomingByte2 = incomingByte2-48;
               else
                   incomingByte2 = 10 + (incomingByte2 - 65); //Ascii A is 65
    
               //calculate % value of the control command
               int control = incomingByte1 *16 + incomingByte2;


               /* Serial.print("vailable\t"); */
               /* Serial.println(Serial.available(),DEC); */
               /* Serial.print("Byte1:\t"); */
               /* Serial.println(incomingByte1,DEC); */
               /* Serial.print("Byte2:\t"); */
               /* Serial.println(incomingByte2,DEC);         */
               /* Serial.print("control: "); */
               /* Serial.println(control,DEC); */

               //use the value received from pc to control luminaire
               /* analogWrite(ledLight, control);  */
               outputLEDValue=control;
           }
           

           //clean buffer
           while(Serial.available()){
               Serial.read();
           }

           
           break;
       }
   case 3: //PID
       {
           digitalWrite(redPin, HIGH);
           digitalWrite(greenPin, LOW);
           digitalWrite(bluePin, LOW);

           // writes latest PID value to fan
           analogWrite(fanPin,255);

           // writes latest PID value to luminaire
           /* analogWrite(ledLight,pidLED_u); */
           outputLEDValue=pidLED_u;
     
           break;
       }
   default: //=OFF
       {
           digitalWrite(redPin, LOW);
           digitalWrite(greenPin, LOW);
           digitalWrite(bluePin, LOW);
           outputLEDValue=0;
           break;
       }
   }
   analogWrite(ledLight,outputLEDValue);

}


