/*
  Tips for Using Plantower PMS7003 air quality sensor to a Particle IoT microcontroller
  Made by 2016 winduino 
  http://wemakers.tistory.com  
    
*/
#include "UbidotsMicroESP8266.h"
#include "DHT.h"
#include <SoftwareSerial.h> 

//ubidots에 연결에 필요한 토큰, 와이파이 등
#define ID_RELAY  "5a8ce67cc03f97499afd618a"  // my-data-source  5a77e5a0c03f972bb1fecd9a
//#define ID_RELAY  "5a8bef7e76254244191d794c"  // mydataSource  5a8bef7e76254244191d794c
                                                // 5a8ce67cc03f97499afd618a
#define TOKEN  "A1E-EdyadCTkxS9tPpVRitysICswqwKZcZ"  // Put here your Ubidots TOKEN

//#define WIFISSID "ollehEgg_440" // Put here your Wi-Fi SSID
//#define PASSWORD "info27640" // Put here your Wi-Fi password

//#define WIFISSID "CJWiFi_F91C" // Put here your Wi-Fi SSID
//#define PASSWORD "8001003032" // Put here your Wi-Fi password

#define WIFISSID "iptime" // Put here your Wi-Fi SSID
#define PASSWORD "" // Put here your Wi-Fi password 

#define ID_PM1_0 "5a7d92fcc03f972312a9c333" // PM1.0 ID값 입력
#define ID_PM2_5 "5a7d931bc03f9722f54ce8f2" // PM2.5 ID값 입력
#define ID_PM10  "5a7d932ac03f972441c281f3" // PM10 ID값 입력

#define ID_Temperature "5a76dc11c03f9725f7b7906a" // 온도 ID값 입력
#define ID_Humidity "5a76dbd7c03f9725dad4c10e" // 습도 ID값 입력

#define RELAY D4  // relay pin
 
#define DHTPIN D7    // what pin we're connected to
#define DHTTYPE DHT11   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE,11);

Ubidots client(TOKEN);

// D5 -> GPIO14
// D6 -> GPIO12
//SoftwareSerial mySerial(12, 14); // RX, TX 
SoftwareSerial mySerial(D6, D5); // RX, TX 
unsigned char  pms[32]; 

char _h[5], _t[5];

//#define DEBUG  
#define  MEAN_NUMBER  10
#define  MAX_PM   0
#define  MIN_PM   32767

int incomingByte = 0; // for incoming serial data
const int MAX_FRAME_LEN = 64;
char frameBuf[MAX_FRAME_LEN];
int detectOff = 0;
int frameLen = MAX_FRAME_LEN;
bool inFrame = false;
char printbuf[256];
unsigned int calcChecksum = 0;
unsigned int pm1_0=0, pm2_5=0, pm10_0=0;
unsigned int tmp_max_pm1_0, tmp_max_pm2_5, tmp_max_pm10_0; 
unsigned int tmp_min_pm1_0, tmp_min_pm2_5, tmp_min_pm10_0; 
byte i=0;

struct PMS7003_framestruct {
    byte  frameHeader[2];
    unsigned int  frameLen = MAX_FRAME_LEN;
    unsigned int  concPM1_0_CF1;
    unsigned int  concPM2_5_CF1;
    unsigned int  concPM10_0_CF1;
    unsigned int  checksum;
} thisFrame;

void setup() {
    Serial.begin(57600);
    mySerial.begin(9600); 
    client.wifiConnection(WIFISSID, PASSWORD);

    // relay pinMode
    pinMode(RELAY, OUTPUT);
    
    Serial.println("DHT11 test!");
    dht.begin();  
    delay(1000);
    Serial.println("-- Initializing...");
    Serial.print("Connecting to ");
    Serial.println(WIFISSID);
   
   while (WiFi.status() != WL_CONNECTED) 
   {
     delay(500);
     Serial.print(".");
   }
   Serial.println("");
   Serial.println("WiFi connected");
}


bool pms7003_read() {
#ifdef DEBUG  
    Serial.println("----- Reading PMS7003 -----");
#endif
    mySerial.begin(9600);
    bool packetReceived = false;
    calcChecksum = 0;
    while (!packetReceived) {
        if (mySerial.available() > 32) {
            int drain = mySerial.available();
#ifdef DEBUG
                Serial.print("----- Draining buffer: -----");
                Serial.println(mySerial.available(), DEC);
#endif
            for (int i = drain; i > 0; i--) {
                mySerial.read();
            }
        }
        if (mySerial.available() > 0) {
#ifdef DEBUG
                Serial.print("----- Available: -----");
                Serial.println(mySerial.available(), DEC);
#endif
            incomingByte = mySerial.read();
#ifdef DEBUG
                Serial.print("----- READ: -----");
                Serial.println(incomingByte, HEX);
#endif
            if (!inFrame) {
                if (incomingByte == 0x42 && detectOff == 0) {
                    frameBuf[detectOff] = incomingByte;
                    thisFrame.frameHeader[0] = incomingByte;
                    calcChecksum = incomingByte; // Checksum init!
                    detectOff++;
                }
                else if (incomingByte == 0x4D && detectOff == 1) {
                    frameBuf[detectOff] = incomingByte;
                    thisFrame.frameHeader[1] = incomingByte;
                    calcChecksum += incomingByte;
                    inFrame = true;
                    detectOff++;
                }
                else {
                    Serial.print("----- Frame syncing... -----");
                    Serial.print(incomingByte, HEX);
                    Serial.println();
                }
            }
            else {
                frameBuf[detectOff] = incomingByte;
                calcChecksum += incomingByte;
                detectOff++;
                unsigned int  val = (frameBuf[detectOff-1]&0xff)+(frameBuf[detectOff-2]<<8);
                switch (detectOff) {
                    case 4:
                        thisFrame.frameLen = val;
                        frameLen = val + detectOff;
                        break;
                    case 6:
                        thisFrame.concPM1_0_CF1 = val;
                        break;
                    case 8:
                        thisFrame.concPM2_5_CF1 = val;
                        break;
                    case 10:
                        thisFrame.concPM10_0_CF1 = val;
                        break;
                    case 32:
                        thisFrame.checksum = val;
                        calcChecksum -= ((val>>8)+(val&0xFF));
                        break;
                    default:
                        break;
                }
                if (detectOff >= frameLen) {
#ifdef DEBUG          
                    sprintf(printbuf, "PMS7003 ");
                    sprintf(printbuf, "%s[%02x %02x] (%04x) ", printbuf,
                        thisFrame.frameHeader[0], thisFrame.frameHeader[1], thisFrame.frameLen);
                    sprintf(printbuf, "%sCF1=[%04x %04x %04x] ", printbuf,
                        thisFrame.concPM1_0_CF1, thisFrame.concPM2_5_CF1, thisFrame.concPM10_0_CF1);
                    sprintf(printbuf, "%scsum=%04x %s xsum=%04x", printbuf,
                        thisFrame.checksum, (calcChecksum == thisFrame.checksum ? "==" : "!="), calcChecksum);
                    Serial.println(printbuf);
#endif        
                    packetReceived = true;
                    detectOff = 0;
                    inFrame = false;
                }
            }
        }
    }
    //mySerial.end();
    return (calcChecksum == thisFrame.checksum);
}
void loop () {
  /// RELAY
  float value = client.getValue(ID_RELAY);
  Serial.print("Relay Value: ");
  Serial.println(value);

  if ((int)value == 1) {
//    digitalWrite(RELAY, (int)value);
    digitalWrite(RELAY, HIGH);
  } else {
    digitalWrite(RELAY, LOW);
  }
    
  delay(1000);

  ///////////////////////////////////////
//  float h = dht.readHumidity();
//  float t = dht.readTemperature();
//  dtostrf(t, 4, 1, _t);  
//  if (isnan(t) || isnan(h)) 
//  {
//    Serial.println("Failed to read from DHT");
//  } else 
//  {
//    Serial.print("Temp: "); 
//    Serial.print(t);
//    Serial.print("Humid : "); 
//    Serial.print(h);
//    client.add(ID_Temperature, t); //ubidots으로 값을 보내는 method입니다. 
//    client.add(ID_Humidity, h);
//    //첫번째 매개변수에는 온도,습도 변수 ID와  
//    client.sendAll(false);
//   }   
//   delay(1000);
////////////////////////////////////////////////////////////
  if(i==0) { 
    tmp_max_pm1_0  = MAX_PM;
    tmp_max_pm2_5  = MAX_PM;
    tmp_max_pm10_0 = MAX_PM;
    tmp_min_pm1_0  = MIN_PM;
    tmp_min_pm2_5  = MIN_PM;
    tmp_min_pm10_0 = MIN_PM;
  }
  if (pms7003_read()) 
  {
    tmp_max_pm1_0  = _max(thisFrame.concPM1_0_CF1, tmp_max_pm1_0);
    tmp_max_pm2_5  = _max(thisFrame.concPM2_5_CF1, tmp_max_pm2_5);
    tmp_max_pm10_0 = _max(thisFrame.concPM10_0_CF1, tmp_max_pm10_0);
    tmp_min_pm1_0  = _min(thisFrame.concPM1_0_CF1, tmp_min_pm1_0);
    tmp_min_pm2_5  = _min(thisFrame.concPM2_5_CF1, tmp_min_pm2_5);
    tmp_min_pm10_0 = _min(thisFrame.concPM10_0_CF1, tmp_min_pm10_0);
    pm1_0 += thisFrame.concPM1_0_CF1;
    pm2_5 += thisFrame.concPM2_5_CF1;
    pm10_0 += thisFrame.concPM10_0_CF1;
    i++;
    Serial.print("O");
  }
  else {
    Serial.print("*");
  }
  
  if(i==MEAN_NUMBER) 
  {
    sprintf(printbuf, "[Checksum OK]");
    sprintf(printbuf, "%s PM1.0 = %02x, PM2.5 = %02x, PM10 = %02x", printbuf, 
    (pm1_0-tmp_max_pm1_0-tmp_min_pm1_0)/(MEAN_NUMBER-2), 
    (pm2_5-tmp_max_pm2_5-tmp_min_pm2_5)/(MEAN_NUMBER-2), 
    (pm10_0-tmp_max_pm10_0-tmp_min_pm10_0)/(MEAN_NUMBER-2));

    sprintf(printbuf, "%s PM1.0 = %d, PM2.5 = %d, PM10 = %d", printbuf, 
    (pm1_0-tmp_max_pm1_0-tmp_min_pm1_0)/(MEAN_NUMBER-2), 
    (pm2_5-tmp_max_pm2_5-tmp_min_pm2_5)/(MEAN_NUMBER-2), 
    (pm10_0-tmp_max_pm10_0-tmp_min_pm10_0)/(MEAN_NUMBER-2));

    //////////////////////////////////////////////////////////////
    client.add(ID_PM1_0,  (pm1_0-tmp_max_pm1_0-tmp_min_pm1_0)/(MEAN_NUMBER-2));
    client.add(ID_PM2_5,  (pm2_5-tmp_max_pm2_5-tmp_min_pm2_5)/(MEAN_NUMBER-2));
    client.add(ID_PM10, (pm10_0-tmp_max_pm10_0-tmp_min_pm10_0)/(MEAN_NUMBER-2));

    client.sendAll(false);


//  sprintf(printbuf, "%s [max =%d,%d,%d, min = %d,%d,%d]", printbuf,
//    tmp_max_pm1_0, tmp_max_pm2_5, tmp_max_pm10_0,
//    tmp_min_pm1_0, tmp_min_pm2_5, tmp_min_pm10_0);  

//  sprintf(printbuf, "%s [max =%02x,%02x,%02x, min = %02x,%02x,%02x]", printbuf,
//    tmp_max_pm1_0, tmp_max_pm2_5, tmp_max_pm10_0,
//    tmp_min_pm1_0, tmp_min_pm2_5, tmp_min_pm10_0);  

//    sprintf(printbuf, "%s [max =%dx,%d,%d, min = %d,%d,%d]", printbuf,
//    tmp_max_pm1_0, tmp_max_pm2_5, tmp_max_pm10_0,
//    tmp_min_pm1_0, tmp_min_pm2_5, tmp_min_pm10_0);  
//    Serial.println();
//    Serial.println(printbuf);
    pm1_0=pm2_5=pm10_0=i=0;
    delay(1000);

/////////////////////////////

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  dtostrf(t, 4, 1, _t);  
  if (isnan(t) || isnan(h)) 
  {
    Serial.println("Failed to read from DHT");
  } else 
  {
    Serial.print("Temp: "); 
    Serial.print(t);
    Serial.print("Humid : "); 
    Serial.print(h);
    client.add(ID_Temperature, t); //ubidots으로 값을 보내는 method입니다. 
    client.add(ID_Humidity, h);
    //첫번째 매개변수에는 온도,습도 변수 ID와  
    client.sendAll(false);
   }   
   delay(1000);
  }
}



