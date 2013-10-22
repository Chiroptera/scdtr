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
* PID variables
**********************************************************/

unsigned long pid_lastTime; //for constant sample time assurance
int pid_sampleTime = 10;
double pid_y, pid_u; //input reading and output value
double pid_ref=25; //reference
double pid_errorSum, pid_lastError; //error sum and previous erros for integral and derivative components
double pid_kp=1, pid_ki=0, pid_kd=100; //proportional, integral and derivative variables


//    P-Control: P=0.50*Gu, I=0, D=0.
//    PI-Control: P=0.45*Gu, I=1.2/tu, D=0.
//    PID-Control: P=0.60*Gu, I=2/tu, D=tu/8. 


void pid(){
//pid_y=analogRead(PIN_Y);
pid_y=tempAverage;

unsigned long now = millis();
double timeChange = (double) (now - pid_lastTime);
if (timeChange < pid_sampleTime) return; //don't compute output if sample time has not been reached
double error = pid_ref - pid_y; //compute current error
pid_errorSum += (error * pid_sampleTime); //error's integral
int errorDerivative = (error - pid_lastError) / pid_sampleTime; //error's derivative
pid_u = pid_kp * error + pid_ki * pid_errorSum + pid_kd * errorDerivative;
pid_lastError=error;
pid_lastTime=now;
//map pid_u
//analogWrite(PIN_U,pid_u)

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


Serial.print("\t Temperature:");
Serial.print(tempAverage);
Serial.println();




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

  pid();

  int potentiometerFanValue = analogRead(potentiometerFanPin);
  potentiometerFanValue=map(potentiometerFanValue,0,1023,0,255);
  analogWrite(fanPin,pid_u);
  
 Serial.print("Pot fan:");
 
 Serial.println(pid_u,DEC);
  
  
/**********************************************************
*
* DRIVING THE LED
*
**********************************************************/
  
  int potentiometerLEDValue = analogRead(potentiometerLEDPin);
  luminaireValue=map(potentiometerLEDValue,0,1023,0,255);
  analogWrite(ledLight,luminaireValue);
  
 //Serial.print("Pot luminaire:");
 //Serial.println(potentiometerLEDValue,DEC);
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

  delay(100);

}
