//CO2 Sensor 
//Starting from closest to the edge, leave the first pin unattached
//2nd pin = on breadboard, connected with GND cable to GND pin on Mega and GND cable connected to battery
//3rd pin = on breadboard, connected with GND cable connected to battery
//4th pin = pin 5 on Arduino TX3
//5th pin = pin 7 on Arduino RX3

//PM Sensor
//Starting from upper row with pins facing you?? maybe
//1st pin = + terminal on breadboard
//2nd pin = - terminal on breadboard
//3rd pin = unused
//4th pin = pin 6 on Arduino TX2
//5th pin = pin 8 on Arduino RX2

//SHT15TEM&HUM sensor
//starting from left with sensor facing you
//1st pin = SCL ( PIN 21) on mega
//2nd pin = + terminal on breadboard
//3rd pin = - terminal on breadboard 
//4rd pin = SDA ( PIN 20) on mega

#include <Arduino.h>
#include "serialReadPMValue.h"
#include <Wire.h>
//#include <SPI.h>

//K30 
//This sensor has automatic baseline correction (ABC), see here https://www.co2indicator.nl/documentatie/ABC-calibration.pdf
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response
//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;
//initializes array with length 12 in order to average readings for 5 minutes
float co2_Conc [12]= {};


//PM
#include "Plantower_PMS7003.h" // at https://github.com/jmstriegel/Plantower_PMS7003
char output[256];
Plantower_PMS7003 pms7003 = Plantower_PMS7003();
int PM01Value=0;          //define PM1.0 value of the air detector module
int PM2_5Value=0;         //define PM2.5 value of the air detector module
int PM10Value=0;         //define PM10 value of the air detector module
//initializes array with length 12 in order to average readings for 5 minutes
float PM01_Conc [12]= {};
float PM2_5_Conc [12]= {};
float PM10_Conc [12]= {};

//SHT15 Temp and Humidity 
//https://create.arduino.cc/projecthub/rafitc/sht85-with-arduino-950fa2
#include "SHTSensor.h" // at https://github.com/Sensirion/arduino-sht
SHTSensor sht(SHTSensor::SHT3X);
float tempF = 0;
float humidity = 0;

//TGS2602
//sensor input PIN
int mqInput = A0;
//pull-down resistor value
int mqR = 10000;
//rO sensor value
long rO = 228607;
//min value for Rs/Ro
float minRsRo_H2S = 0.2031543;
float minRsRo_C7H8 = 0.1877493;
float minRsRo_C2H5OH = 0.188172;
float minRsRo_NH3 = 0.2414247; 
//max value for Rs/Ro
float maxRsRo_H2S = 1.277054;
float maxRsRo_C7H8 = 1.118885;
float maxRsRo_C2H5OH = 1.08456;
float maxRsRo_NH3 = 0.9790021;
//sensor a coefficient value
float a_H2S = 0.1572179;
float a_C7H8 = 1.238677;
float a_C2H5OH = 1.170726;
float a_NH3 = 0.9497495;
//sensor b coefficient value
float b_H2S = -1.850141;
float b_C7H8 = -1.905454;
float b_C2H5OH = -1.941796;
float b_NH3 = -2.429468;
//initializes array with length 12 in order to average readings for 5 minutes
float H2S_Conc [12] = {};
float C7H8_Conc [12] = {};
float C2H5OH_Conc [12] = {};
float NH3_Conc [12] = {};
float H2S_ppm;
float C7H8_ppm;
float C2H5OH_ppm;
float NH3_ppm;

//initializes array with length 12 in order to average readings for 5 minutes
float TEMPF_Conc [12]= {};
float HUM_Conc [12]= {};

//General
//this sets up counter for adding concentrations to the array
int i = 0;

void setup()
{
  //General
  Serial.begin(9600);
  Wire.begin();

  //K30 Ozone
  Serial3.begin(9600);
  
  //SHT85
if (sht.init()) {
  //printnothing
  } else {
    Serial.print(" error");
    }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
  
  //PM
  Serial2.begin(9600);
  pms7003.init(&Serial2);

  //TGS2602
  pinMode(mqInput, INPUT);

}

void loop()
{

  //K30
  sendRequest(readCO2);
  unsigned long valCO2 = getValue(response);

  //PM
   pms7003.updateFrame();
  while (!pms7003.hasNewData()) {
    pms7003.updateFrame();
    delay(10);
    }
 
  PM01Value=pms7003.getPM_1_0(); 
  pms7003.getPM_1_0_atmos();
  PM2_5Value=pms7003.getPM_2_5();
  pms7003.getPM_2_5_atmos();
  PM10Value=pms7003.getPM_10_0(); 
  pms7003.getPM_10_0_atmos(); 

  //SHT 15 tem and humidity sensor
  while (!sht.readSample()) {
    delay(10);
  }
  tempF = (sht.getTemperature() * 9/5) + 32;
  humidity = sht.getHumidity();  

  //TGS2602
  int adcRaw = analogRead(mqInput);
  long rS = ((1024.0 * mqR) / adcRaw) - mqR;
  float rSrO = (float)rS / (float)rO;
 //H2S
  if(rSrO < maxRsRo_H2S && rSrO > minRsRo_H2S) {
 H2S_ppm = a_H2S * pow((float)rS / (float)rO, b_H2S);
  } else {
 Serial.println("H2S Out of range.");
  }
  //C7H8
   if(rSrO < maxRsRo_C7H8 && rSrO > minRsRo_C7H8) {
 C7H8_ppm = a_C7H8 * pow((float)rS / (float)rO, b_C7H8);
  } else {
 Serial.println("C7H8 Out of range.");
  }
  //C2H5OH
   if(rSrO < maxRsRo_C2H5OH && rSrO > minRsRo_C2H5OH) {
 C2H5OH_ppm = a_C2H5OH * pow((float)rS / (float)rO, b_C2H5OH);
  } else {
 Serial.println("C2H5OH Out of range.");
  }
  //NH3
   if(rSrO < maxRsRo_NH3 && rSrO > minRsRo_NH3) {
 NH3_ppm = a_NH3 * pow((float)rS / (float)rO, b_NH3);
  } else {
 Serial.println("NH3 Out of range.");
  }

  //Getting Averages
   if (i<12){ 
  co2_Conc[i] = valCO2;
  PM01_Conc[i] = PM01Value;
  PM2_5_Conc[i] = PM2_5Value;
  PM10_Conc[i] = PM10Value;
  TEMPF_Conc [i] = tempF;
  HUM_Conc [i] = humidity;
  H2S_Conc [i] = H2S_ppm;
  C7H8_Conc [i] = C7H8_ppm;
  C2H5OH_Conc [i] = C2H5OH_ppm;
  NH3_Conc [i] = NH3_ppm;
  
        i++;
  delay (30000);
    }
    
    else{
    float CO2Average = averageValue(co2_Conc, i );  // average will be fractional, so float may be appropriate.
    float PM01Average = averageValue(PM01_Conc, i );
    float PM2_5Average = averageValue(PM2_5_Conc, i );
    float PM10_Average = averageValue(PM10_Conc, i );
    float tempfAverage = averageValue(TEMPF_Conc, i );
    float humAverage = averageValue(HUM_Conc, i );
    float H2SAverage = averageValue(H2S_Conc, i );
    float C7H8Average = averageValue(C7H8_Conc, i );
    float C2H5OHAverage = averageValue(C2H5OH_Conc, i );
    float NH3Average = averageValue(NH3_Conc, i );

 Serial.print(CO2Average);
 Serial.print(", ");
 Serial.print(PM01Average);
 Serial.print(", ");
 Serial.print(PM2_5Average);
 Serial.print(", ");
 Serial.print(PM10_Average);
 Serial.print(", ");
 Serial.print(tempfAverage);
 Serial.print(", ");
 Serial.print(humAverage);
 Serial.print(", ");
 Serial.print(H2SAverage);
 Serial.print(", ");
 Serial.print(C7H8Average);
 Serial.print(", ");
 Serial.print(C2H5OHAverage);
 Serial.print(", ");
 Serial.println(NH3Average);

//Resetting counter
    i = 0;
    }
  }

//K30
void sendRequest(byte packet[])
{
  while(!Serial3.available())  //keep sending request until we start to get a response
  {
    Serial3.write(readCO2,7);
    delay(50);
  }
  
  int timeout=0;  //set a timeoute counter
  while(Serial3.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;  
    if(timeout > 10)    //if it takes to long there was probably an error
      {
        while(Serial3.available())  //flush whatever we have
          Serial3.read();
          
          break;                        //exit and try again
      }
      delay(50);
  }
  
  for (int i=0; i < 7; i++)
  {
    response[i] = Serial3.read();
  }  
}

unsigned long getValue(byte packet[])
{
    int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
    int low = packet[4];                         //low byte for value is 5th byte in the packet

  
    unsigned long val = high*256 + low;                //Combine high byte and low byte with this formula to get value
    return val* valMultiplier;
}

//SHT85 temp and humidy sensor reading
void readTnH()
{
  while (sht.readSample()) {
  // Read values from the sensor
  tempF = (sht.getTemperature() * 9/5) + 32;
  humidity = sht.getHumidity();  
  }/* else {
  tempC = 1;
  tempF = 1;
  humidity_ = 1;
  } */
}

double averageValue(float arrayOfReadings[], int timeinterval){
  float sum = 0;
  for (int j = 0 ; j < timeinterval ; ++j)
    sum += arrayOfReadings[j];
    float avg = ((float) sum) / timeinterval;
    return avg;
  for(int k = 0; k < timeinterval;  ++k)
    arrayOfReadings[k] = (char)0;    
}
