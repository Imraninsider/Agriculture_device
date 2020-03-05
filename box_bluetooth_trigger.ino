#include <SoftwareSerial.h>
//#include "SoftwareI2C.h"
#include <TH02_dev.h>
#include <MutichannelGasSensor.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Arduino.h>
#define LENG 31                 //0x42 + 31 bytes equal to 32 bytes
#define receiveDatIndex 24
#define GAS_EN      1
#define TH02_EN     1
#define MQ135_DEFAULTPPM 399    //default ppm of CO2 for calibration
#define MQ135_RO 68550          //default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682   //CO2 gas value
#define MQ135_EXPONENT -2.769034857       //CO2 gas value
#define MQ135_MAXRSRO 2.428     //for CO2
#define MQ135_MINRSRO 0.358     //for CO2


//####################for bluetooth triggering#################
String myString; 
int dlay = 5000; 


void ftoa(float n, char *res, int afterpoint) ;

// float to string

char *f2s(float f, int p)
{
  char * pBuff;                         // use to remember which part of the buffer to use for dtostrf
  const int iSize = 10;                 // number of bufffers, one for each float before wrapping around
  static char sBuff[iSize][20];         // space for 20 characters including NULL terminator for each float
  static int iCount = 0;                // keep a tab of next place in sBuff to use
  pBuff = sBuff[iCount];                // use this buffer
  if(iCount >= iSize -1)                // check for wrap
  {                                      
    iCount = 0;                         // if wrapping start again and reset
  }
  else
  {
    iCount++;                           // advance the counter
  }
  return dtostrf(f, 0, p, pBuff);       // call the library function
}

int state = 0;
const int pin_scl = 2;      // select a pin as SCL of software I2C
const int pin_sda = 3;      // select a pin as SDA of software I2C
/*
    This is a demo to test gas library
    This code is running on Xadow-mainboard, and the I2C slave is Xadow-gas
    There is a ATmega168PA on Xadow-gas, it get sensors output and feed back to master.
    the data is raw ADC value, algorithm should be realized on master.

    please feel free to write email to me if there is any question

    Jacky Zhang, Embedded Software Engineer
    qi.zhang@seeed.cc
    17,mar,2015
*/

void printDateTime();

RTC_DS1307 rtc;
uint8_t receiveDat[receiveDatIndex]; //receive data from the air detector module
long int sl=1;
int pin=35;

File myFile;
int sensorValue;
int pin44 = 44;

SoftwareSerial BTSerial(10,9); //Rx  | Tx
int t=0;
unsigned char buf[LENG];

int PM01Value=0;          //define PM1.0 value of the air detector module
int PM2_5Value=0;         //define PM2.5 value of the air detector module
int PM10Value=0;

void setup()
{ //Serial1.begin(9600);
  Serial.begin(115200);   //set the serial's Baudrate of the air detector module  
  Serial.println("Started..!");
  if (!SD.begin(53)) 
    {
    Serial.println("initialization failed!");
    }
   // Serial.begin(9600);  // start serial for output
    //Serial.println("Started..!");  
    //Serial.println("Testing!!");
    gas.begin(0x04);//the default I2C address of the slave is 0x04
                    // 0x19 - GREEN BOX, 0x04 - WHITE BOX
    //Serial.println("OK!");
    gas.powerOn();
    //gas.getVersion();
    
    Serial1.begin(9600);   //set the serial1's Baudrate of the air detector module
    BTSerial.begin(9600); // HC-05 default speed in AT command more
    //delay(150);
    TH02.begin(pin_scl, pin_sda);
    Serial.print("runing");
    delay(100);

    pinMode(pin44, OUTPUT);
    
    
    
    rtc.begin();
      //rtc.begin();
    if (! rtc.isrunning()) 
    {
    Serial.println("RTC is NOT running!");
    }
    
    Serial.println("Started..!");  
  
      
}
void loop()
{
  if(sl==1)
  {
    //rtc.begin();
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  //Adjust date and time 
  }
char result[50];
  myFile = SD.open("New_Box.csv", FILE_WRITE);
    
     float no2,co,co2;    
     static float temp,temp1, humi,humi1;
     char test[160],test1[160],ln[6];
            
          printDateTime();    //Print date and time
          Serial.print(", ");
          myFile.print(", ");
          Serial.print(sl++);   //Print Serial Number
          myFile.print(sl);
          Serial.print(", ");
          myFile.print(", ");
             
            if(Serial1.find(0x42))    //start to read when detect 0x42
            {    
              //Serial.print("Testing2!!");
              Serial1.readBytes(buf,LENG);
              //Serial.print("OK2!!");
              if(buf[0] == 0x4d)
              {
                if(checkValue(buf,LENG))
                {
                  PM01Value=transmitPM01(buf); //count PM1.0 value of the air detector module
                  PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
                  PM10Value=transmitPM10(buf); //count PM10 value of the air detector module 
                } 
                        
              }  
            }
         
              
          Serial.print(PM01Value);
          myFile.print(PM01Value);
          Serial.print(", ");
          myFile.print(", ");
          //Serial.println("  ug/m3");            
    
          //Serial.print("PM2.5: ");  //send PM1.0 data to bluetooth
          Serial.print(PM2_5Value);
          
          myFile.print(PM2_5Value);
          Serial.print(", ");
          myFile.print(", ");
         
          //Serial.println("  ug/m3");     
      
          //Serial.print("PM10:  ");  //send PM1.0 data to bluetooth
          Serial.print(PM10Value);
          myFile.print(PM10Value);
          Serial.print(", ");
          myFile.print(", ");
          //Serial.println("  ug/m3");   
          //FOR NO2.................
          no2 = gas.measure_NO2();
          //Serial.print("The concentration of NO2 is ");
          if(no2>=0)
          {   
            Serial.print(no2);
            myFile.print(no2);
          }
          else 
            Serial.print("invalid");
            
          Serial.print(", ");
          myFile.print(", ");
          
          //FOR CO2..............                   
          sensorValue = analogRead(0);       // read analog input pin 1
          
          long resvalue=((float)22000*(1023-sensorValue)/sensorValue);
          //Serial.print("CO2 = ");
          co2=mq135_getppm(resvalue,MQ135_RO);
          Serial.print(mq135_getppm(resvalue,MQ135_RO));
          myFile.print(mq135_getppm(resvalue,MQ135_RO));
          long mq135_getro(int resvalue,long ppm);
          
          Serial.print(", ");
          myFile.print(", ");

          //FOR CO..................
          co = gas.measure_CO();
          //Serial.print("The concentration of CO is ");
          if(co>=0)
          { 
            Serial.print(co);
            myFile.print(co);
          }
          else 
            Serial.print("invalid");
            
          Serial.print(", ");
          myFile.print(", ");       
          
         // Temparature & Humidity

    
          float humidity = TH02.ReadHumidity();
    
          Serial.print(humidity);
          myFile.print(humidity);
      
          Serial.print(", ");
          myFile.print(", ");
          
          float temper = TH02.ReadTemperature();
   
          Serial.println(temper);
          myFile.println(temper);
    
        //For Bluetooth 
      
          if(BTSerial.available() > 0)
          { 
             myString = BTSerial.readString();
             String head = myString.substring(0,8);// Reads the data from the serial port
             if(head == "setdtime"){
                setrtc();
             }
             else if(head == "setdelay")
             {
              int temp = myString.substring(9,11).toInt();
              dlay = temp*1000;
             }
          } 
          //sprintf(result, "%f", PM01Value);
          itoa(PM01Value, result, 10);
          BTSerial.write(result);
          //sprintf(result, "%f", PM2_5Value);
          itoa(PM2_5Value, result, 10);
          BTSerial.write(", ");
          BTSerial.write(result);
          BTSerial.write(", ");
          itoa(PM10Value, result, 10);
          BTSerial.write(result);
          BTSerial.write(", ");
          /*float NO2;
          NO2=no2*100;
          int flg=0;
          if(NO2>100)
          {
            int t1=NO2/100;
            int g=t1*100;
            int t=NO2-g;
            flg=1;
          }
          else if(NO2<10)
          {
            BTSerial.write("0.0");
          }
          else
          {
            BTSerial.write("0.");
          }*/
          ftoa(no2, result, 2);
          BTSerial.write(result);
          /*if(flg==1)
          {
            BTSerial.write(".");
            itoa(t,result,10);
            BTSerial.write(result);
            flg=0;
          }*/
          //gcvt(no2,10,result);

          BTSerial.write(", ");
          ltoa(co2, result, 10);
          BTSerial.write(result);
          BTSerial.write(", ");
          itoa(co, result, 10);
          BTSerial.write(result);
          BTSerial.write(", ");
          itoa(humidity, result, 10);
          BTSerial.write(result);
          BTSerial.write(", ");
          itoa(temper, result, 10);
          BTSerial.write(result);
          BTSerial.println();      
      
          delay(dlay);
          myFile.close();
      
  }
  
void printDateTime()
{
    DateTime now = rtc.now();

    char t_buf[20];
    char d_buf[20];

    sprintf(d_buf, "%04u/%02u/%02u",now.year(), now.month(), now.day());
    Serial.print(d_buf);
    myFile.print(d_buf);
    BTSerial.write(d_buf);
    Serial.print(", ");
    myFile.print(", ");
    BTSerial.write(", ");
    sprintf(t_buf, "%02u:%02u:%02u",now.hour(), now.minute(), now.second());
    Serial.print(t_buf);
    myFile.print(t_buf);
    BTSerial.write(t_buf);
    BTSerial.write(", ");
   
}

// reverses a string 'str' of length 'len' 
void reverse(char *str, int len) 
{ 
    int i=0, j=len-1, temp; 
    while (i<j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; j--; 
    } 
} 
  
 // Converts a given integer x to string str[].  d is the number 
 // of digits required in output. If d is more than the number 
 // of digits in x, then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) 
    { 
        str[i++] = (x%10) + '0'; 
        x = x/10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
// Converts a floating point number to string. 
void ftoa(float n, char *res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) 
    { 
        res[i] = '.';  // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter is needed 
        // to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 
double mq135_getppm(long resvalue,long default_RO) 
{
  double ret = 0;
  double validinterval = 0;
  validinterval = resvalue/(double)default_RO;
  //if(validinterval<MQ135_MAXRSRO && validinterval>MQ135_MINRSRO) {
    ret = (double)MQ135_SCALINGFACTOR * pow( ((double)resvalue/default_RO), MQ135_EXPONENT);
 // }
  ret=ret*100;
  return (ret);
}

char checkValue(unsigned char *thebuf, char leng)
{  
  char receiveflag=0;
  int receiveSum=0;

  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;
 
  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data 
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

//transmit PM10 Value to PC
int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}

//transmit PM2_5 Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
  }

//transmit PM_10 Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module  
  return PM10Val;
}

long mq135_getro(long resvalue,long ppm)
{
  return (long)(resvalue * exp( log(MQ135_SCALINGFACTOR/ppm) / MQ135_EXPONENT ));
}



//####for rtc bluetooth triggering############################
void setrtc()
{
//String head = myString.substring(0,3);
      
     String ds = myString.substring(9,11);
     String mnths = myString.substring(11,13);
     String yrs = myString.substring(13,17);
     String hrs = myString.substring(18,20);
     String ms = myString.substring(20,22);
     String ss = myString.substring(22,24);
     int yr = yrs.toInt();
     int mnth = mnths.toInt();
     int d = ds.toInt();
     int hr = hrs.toInt();
     int m = ms.toInt();
     int s = ss.toInt();
     //for rtc time settiing
    rtc.adjust(DateTime(yr, mnth, d, hr, m, s));
    BTSerial.print("Date Time updated");
     
}
