//******************************

//SETUP
//External Power (5V)
// - red wire = + terminal on breadboard
// - black wire = - terminal on breadboard

//CO2 Sensor 
//Starting from closest to the edge, leave the first pin unattached
//2nd pin = on breadboard, connected with GND cable to GND pin on Mega and GND cable connected to battery
//3rd pin = on breadboard, connected with GND cable connected to battery
//4th pin = pin 5 on Arduino TX3
//5th pin = pin 7 on Arduino RX3

//Ozone Sensor
//Starting from left to right with blue box on the left side
//1st pin = GND = - terminal on breadboard
//2nd pin = AOUT = A1
//3rd pin = DOUT = pin 42
//4th pin = VCC = + terminal on breadboard


//PM Sensor
//Starting from upper row with pins facing you?? maybe
//1st pin = + terminal on breadboard
//2nd pin = - terminal on breadboard
//3rd pin = unused
//4th pin = pin 6 on Arduino TX2
//5th pin = pin 8 on Arduino RX2

//Clock
//https://howtomechatronics.com/tutorials/arduino/arduino-ds3231-real-time-clock-tutorial/
//Only use pins on the right side (right side is when you look at clock with text and battery facing upright)
//Starting from top 
//1st pin (GND) = - terminal on breadboard
//2nd pin = + terminal on breadboard
//3rd pin (SCL) = SCL pin on Arduino
//4th pin (SDA) = SDA pin on Arduino

//HCHO Sensor SEN0231 is used to detect formaldehyde
//starting from left with sensor facing you
//1st pin (GND) = - terminal on breadboard
//2nd pin = + terminal on breadboard
//3rd pin  = Analog 3 on Arduino

//TGS2602
//starting from left with sensor facing you
//1st pin = + terminal on breadboard
//2nd pin = unused
//3nd pin = Analog 0 on Arduino
//4rd pin = - terminal on breadboard
//^^^ FIX THIS... ?

//SHT15TEM&HUM sensor
//starting from left with sensor facing you
//1st pin = SCL ( PIN 21) on mega
//2nd pin = + terminal on breadboard
//3rd pin = - terminal on breadboard 
//4rd pin = SDA ( PIN 20) on mega


 //******************************
#include <Arduino.h>
//#include <SoftwareSerial.h>
#include "serialReadPMValue.h"

#include <Wire.h>
#include <DS3231.h>
//must use DS32321 library from http://www.rinkydinkelectronics.com/library.php?id=73
#include <SPI.h>
#include <SD.h>


//Time
#include "RTClib.h"
DS3231 rtc(SDA, SCL);

//K30 
//SoftwareSerial K_30_Serial(7,5);
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response
//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;
//initializes array with length 60 for each minute in an hour
float co2_Conc [60]= {};

//Ozone
float valO3;
double O3_val;
float O3_Conc [60] = {};
double  ALAOratio_O3;


//PM
#include "Plantower_PMS7003.h" // at https://github.com/jmstriegel/Plantower_PMS7003
char output[256];
Plantower_PMS7003 pms7003 = Plantower_PMS7003();
int PM01Value=0;          //define PM1.0 value of the air detector module
int PM2_5Value=0;         //define PM2.5 value of the air detector module
int PM10Value=0;         //define PM10 value of the air detector module
//initializes array with length 60 for each minute in an hour
float PM01_Conc [60]= {};
float PM2_5_Conc [60]= {};
float PM10_Conc [60]= {};

//SHT15 Temp and Humidity 
//variables for storing values
//https://create.arduino.cc/projecthub/rafitc/sht85-with-arduino-950fa2
#include "SHTSensor.h" // at https://github.com/Sensirion/arduino-sht
//SHTSensor sht;
SHTSensor sht(SHTSensor::SHT3X);
float tempC = 0;
float tempF = 0;
float humidity_ = 0;

//initializes array with length 60 for each minute in an hour
float TEMPC_Conc [60]= {};
float TEMPF_Conc [60]= {};
float HUM_Conc [60]= {};

//HCHO
//define port for HCHO
#define SensorAnalogPin A3  //this pin read the analog voltage from the HCHO sensor
#define VREF  5.0   //voltage on AREF pin
float   valHCHO;
//initializes array with length 60 for each minute in an hour
float HCHO_Conc [60]= {};

//TGS2602 VOC
//define TGS2602 varialble
double   ALAOratio_TGS;
double   C7H8_val;
double   H2S_val;
double   NH3_val;
double   C2H5OH_val;

//initializes array with length 60 for each minute in an hour
float ALAO_Conc_TGS [60]= {};
float ALAO_Conc_O3 [60]= {};

float C7H8_Conc [60]= {};
float H2S_Conc [60]= {};
float NH3_Conc [60]= {};
float C2H5OH_Conc [60]= {};

#define         CALIBRATION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
#define         TGS2602_SENSOR               (0)     //define which analog input channel you are going to use
#define         MQ131_SENSOR                 (1)   //define which analog input channel you are going to use
#define         GAS_CL2                      (0)
#define         GAS_O3                       (1) 

#define         GAS_C7H8                    (18) //Toluene
#define         GAS_H2S                     (19) //Hydrogen Sulfide
#define         GAS_NH3                     (20) //Ammonia
#define         GAS_C2H5OH                   (9) //Alcohol, Ethanol

//this has to be tuned 10K Ohm
float Ro6 = 96843.80;//11972.791;  //TGS2602, average voltage  calculated after place in clean air for 4 hr.
float RL6 = 0.893;        //TGmq136S2602 Gas Sensor V1.3 auto-ctrl.com 

float Ro2 = 17772.17;    //MQ131   2.24 this has to be tuned 10K Ohm
float RL2 = 0.679;   //MQ131   Sainsmart

float percentag_val;
/* The LED to blink. */
const int led = 13;

/*
 The load resistor applied to sensor(-) in series with GND.
 Note that this is hard-coded for my particular setup,
 so you should change it for a different setup.
*/
const double rl = 44.2*1000;

/* The input current. */
const double vc = 5.0;

/* The LED blink period. */
double period = 1000.0;

/*global*/
float           C7H8Curve[2]    =  {0.1319857248,   -1.69516241}; //TGS2602     (0.3;1)( 0.08;10) (0.04;30)
float           H2S_Curve[2]    =  {0.05566582614,-2.954075758}; //TGS2602     (0.8,0.1) (0.4,1) (0.25,3)
float           C2H5OH_quarCurve[2] =  {0.5409499131,-2.312489623};//TGS2602  (0.75,1) (0.3,10) (0.17,30)  
float           NH3_Curve[2]    =  {0.585030495,  -3.448654502  }; //TGS2602   (0.8,1) (0.5,10) (0.3,30) 
float           CL2Curve[2]     =  {56.01727602, -1.359048399};  //MQ131
float           O3Curve[2]      =  {42.84561841, -1.043297135};  //MQ131

//General
//this sets up counter for adding concentrations to the array
int i = 0;

void setup()
{
  //General
  Serial.begin(9600);
  //Serial.println("1");
  //Serial.println("Time,CO2,PM01,PM2.5,PM10,tempf,tempc,hum,TGS_1,C7H8,H2S,NH3,C2H5OH,HCHO,O3_1,O3");
  
  Wire.begin();
  delay(1000);
 
  /*//Time (Initiating RTC)
  rtc.begin();
  rtc.setDOW(WEDNESDAY);
  rtc.setTime(5,42,0);
  rtc.setDate(11, 3, 2020);//date month year
  //Serial.println(now.day);*/

  //K30
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
 // Serial.println("Setup complete.");
  
}

void loop()
{

  //K30
  sendRequest(readCO2);
  //unsigned long valCO2 = getValue(response);
  unsigned long valCO2 = (getValue(response)-237.8)/0.9846;//calibrated

  //PM
  pms7003.updateFrame();
  
  while (!pms7003.hasNewData()) {
    pms7003.updateFrame();
    delay(10);
    }
 
   //if (pms7003.hasNewData()) {
    PM01Value=pms7003.getPM_1_0(); 
    pms7003.getPM_1_0_atmos();
    PM2_5Value=pms7003.getPM_2_5();
    pms7003.getPM_2_5_atmos();
    PM10Value=pms7003.getPM_10_0(); 
    pms7003.getPM_10_0_atmos();
    //Serial.println(" new data get");
   //}
  

   //SHT 15 tem and humidity sensor
 // readTnH();
  while (!sht.readSample()) {
    //Serial.print('.');
    delay(10);
  }
  
  // Read values from the sensor
  tempC = sht.getTemperature();
  tempF = (sht.getTemperature() * 9/5) + 32;
  humidity_ = sht.getHumidity();  
  //Serial.println( "temp hum read");
 
  //TGS 2602 sensor
    while (!MQRead(A0,RL6)>0) {
    delay(10);
    }
  ALAOratio_TGS = MQRead(A0,RL6)/Ro6;
  C7H8_val = MQGetGasPercentage(ALAOratio_TGS,Ro6,GAS_C7H8,TGS2602_SENSOR);
  H2S_val = MQGetGasPercentage(ALAOratio_TGS,Ro6,GAS_H2S,TGS2602_SENSOR);
  NH3_val = MQGetGasPercentage(ALAOratio_TGS,Ro6,GAS_NH3,TGS2602_SENSOR);
  C2H5OH_val = MQGetGasPercentage(ALAOratio_TGS,Ro6,GAS_C2H5OH,TGS2602_SENSOR);

  //HCHO sensor
  valHCHO = analogReadPPM();

  //O3 Sensor
   while (!MQRead(A1,RL2)>0) {
    delay(10);
    }
  ALAOratio_O3 = MQRead(A1,RL2)/Ro2;
  //Serial.print(ALAOratio_O3);
  O3_val = MQGetGasPercentage(ALAOratio_O3,Ro2, GAS_O3,MQ131_SENSOR);

  //Getting Averages
   if (i<30){
    //
    
  //Serial.print(rtc.getDOWStr());
  //Serial.print(" ");
  //Serial.print(rtc.getDateStr());
  //Serial.print(" -- ");
  //Serial.print(rtc.getTimeStr());//
  //Serial.print(" ");

  co2_Conc[i] = valCO2;
  PM01_Conc[i] = PM01Value;
  PM2_5_Conc[i] = PM2_5Value;
  PM10_Conc[i] = PM10Value;
  TEMPC_Conc [i] = tempC;
  TEMPF_Conc [i] = tempF;
  HUM_Conc [i] = humidity_;
  ALAO_Conc_TGS [i] = ALAOratio_TGS;
  C7H8_Conc [i] = C7H8_val;
  H2S_Conc [i] = H2S_val;
  NH3_Conc [i] = NH3_val;
  C2H5OH_Conc [i] = C2H5OH_val;
  HCHO_Conc [i] = valHCHO;
  O3_Conc [i] = O3_val;
  ALAO_Conc_O3 [i] = ALAOratio_O3;
//Serial.print(i);
  
 Serial.print(valCO2);
 Serial.print(" ");
 Serial.print(PM01Value);
 Serial.print(" ");
 Serial.print(PM2_5Value);
 Serial.print(" ");
 Serial.print(PM10Value);
 Serial.print(" ");
 Serial.print(tempC);
 Serial.print(" ");
 Serial.print(tempF);
 Serial.print(" ");
 Serial.print(humidity_);
 Serial.print(" ");
 Serial.print(ALAOratio_TGS);
 Serial.print(" ");
 Serial.print(C7H8_val);
 Serial.print(" ");
 Serial.print(H2S_val);
 Serial.print(" ");
 Serial.print(NH3_val);
 Serial.print(" ");
 Serial.print(C2H5OH_val);
 Serial.println(" ");
 /*Serial.println(valHCHO);
 Serial.print(" ");
 Serial.print(ALAOratio_O3);
 Serial.print(" ");
 Serial.println(valO3);*/
        i++;
// delay(1000);
    }
    
    else{
      
    float CO2Average = averageValue(co2_Conc, i );  // average will be fractional, so float may be appropriate.
    float PM01Average = averageValue(PM01_Conc, i );
    float PM2_5Average = averageValue(PM2_5_Conc, i );
    float PM10_Average = averageValue(PM10_Conc, i );
    float tempcAverage = averageValue(TEMPC_Conc, i );
    float tempfAverage = averageValue(TEMPF_Conc, i );
    float humAverage = averageValue(HUM_Conc, i );
    float ALAOTGSAverage = averageValue(ALAO_Conc_TGS, i);
    float ALAOO3Average = averageValue(ALAO_Conc_O3, i );
    float C7H8Average = averageValue(C7H8_Conc, i );
    float H2SAverage = averageValue(H2S_Conc, i );
    float NH3Average = averageValue(NH3_Conc, i );
    float C2H5OHAverage = averageValue(C2H5OH_Conc, i );
    float HCHOAverage = averageValue(HCHO_Conc, i );
    float O3Average = averageValue(O3_Conc, i );

 //Printing averages to SD Card
 
// myFile = SD.open("Deploy1.txt", FILE_WRITE);
//    if (myFile){ 
/*
 Serial.print(rtc.getDOWStr());
 Serial.print(" ");
 Serial.print(rtc.getDateStr());
 Serial.print("--");
 Serial.print(rtc.getTimeStr());
 Serial.print(" ");
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
 Serial.print(tempcAverage);
 Serial.print(", ");
 Serial.print(humAverage);
 Serial.print(", ");
 Serial.print(ALAOTGSAverage);
 Serial.print(", ");
 Serial.print(C7H8Average);
 Serial.print(", ");
 Serial.print(H2SAverage);
 Serial.print(", ");
 Serial.print(NH3Average);
 Serial.print(", ");
 Serial.print(C2H5OHAverage);
 Serial.print(", ");
 Serial.print(HCHOAverage);
 Serial.print(", ");
 Serial.print(ALAOO3Average);
 Serial.print(", ");
 Serial.print(O3Average);
 Serial.println();*/
// myFile.close();
//    }
//      else {
//    // if the file didn't open, print an error:
//    Serial.println(F("Error opening textfile"));
//  }
    

//Resetting counter
    i = 0;
    }
  // delay(1000);
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
  tempC = sht.getTemperature();
  tempF = (sht.getTemperature() * 9/5) + 32;
  humidity_ = sht.getHumidity();  
  }/* else {
  tempC = 1;
  tempF = 1;
  humidity_ = 1;
  } */
}


//HCHO SENSOR
float analogReadPPM()
{
   float analogVoltage = analogRead(SensorAnalogPin) / 1024.0 * VREF;
   float ppm = 3.125 * analogVoltage - 1.25;  //linear relationship(0.4V for 0 ppm and 2V for 5ppm)
   if(ppm<0)  ppm=0;
   else if(ppm>5)  ppm = 5;
   return ppm;
}

//TGS2602
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin,float rl_value)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    float resis = MQResistanceCalculation(analogRead(mq_pin),rl_value);
    if (resis <= 0){ 
      resis = MQResistanceCalculation(analogRead(mq_pin),rl_value);
      delay(5);
    }
    rs += resis;
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
float MQGetGasPercentage(float rs_ro_ratio, float ro, int gas_id, int sensor_id)
{
  if (sensor_id == TGS2602_SENSOR ){
    if ( gas_id == GAS_C7H8 ) {
      return MQGetPercentage(rs_ro_ratio,ro,C7H8Curve);  //TGS2602
    } else if ( gas_id == GAS_H2S ) {
      return MQGetPercentage(rs_ro_ratio,ro,H2S_Curve);  //TGS2602
    } else if ( gas_id == GAS_NH3 ) {
      return MQGetPercentage(rs_ro_ratio,ro,NH3_Curve);  //TGS2602
    } else if ( gas_id == GAS_C2H5OH ) {
      return MQGetPercentage(rs_ro_ratio,ro,C2H5OH_quarCurve); //TGS2602
    }    
  }else if (sensor_id == MQ131_SENSOR ){
    if ( gas_id == GAS_CL2 ) {
       return MQGetPercentage(rs_ro_ratio,ro,CL2Curve);  //MQ131
    } else if ( gas_id == GAS_O3 ) {
       return MQGetPercentage(rs_ro_ratio,ro,O3Curve);   //MQ131
    }    
  }
  
}
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
float MQGetPercentage(float rs_ro_ratio, float ro, float *pcurve)
{
  float calculatedPercentage= 0;
  //Serial.println(pow(((double)rs_ro_ratio), pcurve[1]));
  calculatedPercentage = pow(((float)rs_ro_ratio), pcurve[1])*pcurve[0];
  //Serial.print(calculatedPercentage);
  return calculatedPercentage;
}


/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage across the load resistor and its resistance, the resistance of the sensor could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc,float rl_value)
{
  //Serial.println(raw_adc);
  //Serial.println(rl_value);
  float resistancecalculation= (1024.*1000.* rl_value)/raw_adc-rl_value;
  //Serial.println(resistancecalculation);
  return  (long)(resistancecalculation);
;
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
