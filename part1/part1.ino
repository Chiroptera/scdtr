// this constant won't change.  It's the pin number
// of the sensor's output:
const int proxSensor = 5;
const int tempSensor = 4;
const int ledLight = 6;

const int LedPresence = 3;       // select the pin for the LED that detects presence
const int LedTemperature = 4;       // select the pin for the LED that detects temperatures above 50ºC

const int potentiometerFanPin = A1;
const int potentiometerLEDPin = A2;

const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;

const int pushbutton1 = 12;
const int pushbutton2 = 13;



//definitions for the LDR
#define LDRpin A0	//input pin for the LDR sensor
#define VoltDiv 10000	//resistance used with the LDR for the voltage divisor
#define LDRvcc 5	//voltage used to supply the voltage divisor
#define LDR_uplimit 20000
#define LDR_downlimit 10000

#define ledLightLow 8	//pin for LED for light off
#define ledLightHigh 7	//pin for LED for light on

int LDRValue = 0;	// variable to store the value coming from the sensor
float LDRVolt =0;	// LDR tension
float LDRes=0;		// LDR resistance



void setup() {
  // initialize serial communication:
  pinMode(proxSensor, INPUT);
  pinMode(LedPresence, OUTPUT);
  pinMode(ledLight, OUTPUT); 

  pinMode(pushbutton1, OUTPUT);
  pinMode(pushbutton2, OUTPUT);
  
  pinMode(tempSensor, INPUT);
  pinMode(LedTemperature, OUTPUT); 

  // declare the LED pins as an OUTPUT:
  pinMode(ledLightLow, OUTPUT); 
  pinMode(ledLightHigh, OUTPUT);  

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  Serial.begin(9600);	//initialize serial communication 
}




void loop()
{


  int proximity, temperature, distance;
  float realtemp;



  ///////////////////////////////////////////////////////////////////
  // proximity sensor
  //digitalWrite(ledLight,HIGH);
  analogWrite(ledLight,220);
  proximity = analogRead(proxSensor);
  distance=-0.146*proximity+63.87;
  if (distance < 50) digitalWrite(LedPresence, HIGH);    // esta ocupado acende o LED
  else  digitalWrite(LedPresence, LOW);                  // livre

  // equation y=-0.146x + 63.87 (in cm); it's linear in the interval [20,50] (cm)


  /*
 Serial.print("proximity:");
   Serial.print(proximity);
   Serial.print("\t   distance:");
   Serial.println(distance);
   */
  ////////////////////////////////////////////////////////////////
  //temperature
  temperature = analogRead(tempSensor);
  realtemp=((5 * (float)temperature)/1023)/0.01;



  if (realtemp > 50) digitalWrite(LedTemperature, HIGH);    // esta ocupado acende o LED
  else  digitalWrite(LedTemperature, LOW);
  /*Serial.print("temp:");
   Serial.print(temperature);
   Serial.print("\t realtemp:");
   Serial.print(realtemp);
   Serial.println();
   */


  ///////////////////////////////////////////////////
  //LDR

  // read the value from the sensor
  LDRValue = analogRead(LDRpin);
  /*Serial.print("Analog reading[LDR]:");
   Serial.println(LDRValue);
   */
  //convert the LDR reading to a voltage
  LDRVolt = (5.00*LDRValue)/1023.00;
  //LDRVolt = map(sensorValue, 0, 1023, 0.00 , 5.00);
  /*Serial.print("Voltage [LDR]:");
   Serial.println(LDRVolt);
   */
  //compute LDR resistance
  LDRes= ( LDRvcc*VoltDiv- LDRVolt*VoltDiv) / LDRVolt;
  /*Serial.print("Resistance [LDR]:");
   Serial.println(LDRes);
   Serial.println("--");
   */
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



  ////////////////////////////////////////////////////////
  // Read potentionmeter

  //NOT WORKING

  int potentiometerFanValue = analogRead(potentiometerFanPin);
  //int potentiometerLEDValue = analaogRead(potentiometerLEDPin);

  //Serial.println(potentiometerFanValue,DEC);





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

  ///////////////////////////////////////////////////////
  // Reading commands

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


////////////////////////////////////////
//pushbuttons

  
  if ( digitalRead(pushbutton1) == HIGH)
      digitalWrite(bluePin, HIGH);
  else
      digitalWrite(bluePin, LOW);

  delay(100);                  

}


