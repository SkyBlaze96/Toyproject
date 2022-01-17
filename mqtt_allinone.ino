#include <DHT.h>
#include <DHT_U.h>
#include <WiFiEsp.h>
#include <PubSubClient.h>
#include <string.h>
#include <MQUnifiedsensor.h>
#include "DFRobotHCHOSensor.h"
#include "SoftwareSerial.h"
#define DHTPIN 2
#define DHTTYPE DHT22
#ifndef HAVE_HWSERIAL1
#endif
#define mqtt_data_size 50
#define RatioMQ135CleanAir 3.6
#define SensorSerialPin 10 // this pin read the uart signal from the HCHO sensor

SoftwareSerial sensorSerial(SensorSerialPin, SensorSerialPin);
DFRobotHCHOSensor hchoSensor(&sensorSerial);
MQUnifiedsensor MQ135("Arduino Mega", 5, 10, A1, "MQ-135");
#include <MHZ19.h> //CO가스 설정
MHZ19 mhz(&Serial3);

const char *ssid = "GGK";
const char *password = "rla870626";
const char *mqttServer = "211.48.228.15"; //broker.hivemq.com
int mqttPort = 1883;
long lastMsg = 0;


int value = 0;
const int led = LED_BUILTIN;
int sub_tol = 1000; //톨루엔 측정주기
int sub_tol_dt = 0;
int sub_nh4 = 1000; //암모니아 측정주기
int sub_nh4_dt = 0;
int sub_ace = 1000; //아세톤 측정주기
int sub_ace_dt = 0;
int sub_co2 = 1000; //이산화탄소 측정주기
int sub_co2_dt = 0;
int sub_co = 1000;  // 일산화탄소 측정주기
int sub_co_dt = 0;
int sub_form = 1000;  // 일산화탄소 측정주기
int sub_form_dt = 0;
int sub_temp = 1000;  // 온도 측정주기
int sub_temp_dt = 0;
int sub_hum = 1000;  // 습도 측정주기
int sub_hum_dt = 0;
String mid = "\"mid_1\"";
int status = WL_IDLE_STATUS;
String packet;
float form;
WiFiEspClient espClient;

PubSubClient mqttClient(espClient);
DHT dht(DHTPIN, DHTTYPE);
// void sendMqtt(float tol, bool tolResult);

int prev_time = 0;
void updateTime()
{
  int now = millis();
  int dt = now - prev_time;
  prev_time = now;
  sub_tol_dt += dt;
  sub_nh4_dt += dt;
  sub_ace_dt += dt;
  sub_co2_dt += dt;
  sub_co_dt  += dt;
  sub_form_dt += dt;
  sub_temp_dt += dt;
  sub_hum_dt += dt;
}

void setupMQTT()
{
  mqttClient.setServer(mqttServer, mqttPort);
  // set the callback function
  mqttClient.setCallback(callback);
}

void reconnect()
{
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected())
  {
    Serial.println("Reconnecting to MQTT Broker..");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str()))
    {
      Serial.println("Connected. mqtt");

      mqttClient.subscribe("sensor/sub_tol");  //톨루엔 주기조정
      mqttClient.subscribe("sensor/sub_nh4");  //암모니아 주기조정
      mqttClient.subscribe("sensor/sub_ace");  //아세톤 주기조정
      mqttClient.subscribe("sensor/sub_co2");  //이산화탄소 주기조정
      mqttClient.subscribe("sensor/sub_co");   //일산화탄소 주기조정
      mqttClient.subscribe("sensor/sub_form"); //포름알데히드 주기조정
    }
  }
  delay(1000);
}

void setup()
{
  pinMode(led, OUTPUT);
  Serial.begin(9600);       //내장 시리얼
  Serial1.begin(9600);      //WiFi 모듈 활용 시리얼
  sensorSerial.begin(9600); //포름알데히드 센서 시리얼
  sensorSerial.listen();
  Serial3.begin(9600); //이산화탄소 센서 시리얼
  WiFi.init(&Serial1);
  dht.begin();
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  MQ135.setR0(calcR0 / 10);

  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true)
      ;
  }

  // attempt to connect to WiFi network
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, password);
  }

  Serial.println("You're connected to the network");

  setupMQTT();
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("Callback");
  Serial.print("leng:");
  Serial.println(length);
  Serial.print("Message:");
  
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }

  int *ptrSubSensor = nullptr;
  String sTopic = String(topic);
  if (sTopic == "sensor/sub_tol")
  {
    ptrSubSensor = &sub_tol;
  }
  if (sTopic == "sensor/sub_nh4")
  {
    ptrSubSensor = &sub_nh4;
  }
  if (sTopic == "sensor/sub_ace")
  {
    ptrSubSensor = &sub_ace;
  }
  if (sTopic == "sensor/sub_co2")
  {
    ptrSubSensor = &sub_co2;
  }
  if (sTopic == "sensor/sub_co")
  {
    ptrSubSensor = &sub_co;
  }
  if (sTopic == "sensor/sub_form")
  {
    ptrSubSensor = &sub_form;
  }
  int t = ((char)payload[0] - '0') * 1000;
  *ptrSubSensor = t;
}

void loop()
{
  if (!mqttClient.connected())
  {
    reconnect();
  }
  updateTime();
  MQ135.update();
  MHZ19_RESULT response = mhz.retrieveData(); //CO2가스


  mqttClient.loop();


  float tol,nh4,ace,co2,co,form,temp,hum;
  bool tolResult = getTolue(&tol);
  bool nh4Result = getNh4(&nh4);
  bool aceResult = getAce(&ace);
  bool co2Result = getCo2(&co2);
  bool coResult = getCo(&co);
  bool formResult = getForm(&form);
  bool tempResult = getTemp(&temp);
  bool humResult = getHumidity(&hum);
  sendMqtt(tol, tolResult, nh4,nh4Result,ace,aceResult,co2,co2Result,co,coResult,form,formResult,temp,tempResult,hum,humResult);

}

void sendMqtt(
  float tol, bool tolResult,
  float nh4, bool nh4Result,
  float ace,bool aceResult,
  float co2, bool co2Result, 
  float co, bool coResult,
  float form, bool formResult,
  float temp, bool tempResult,
  float hum, bool humResult
)
{
  String payload = "{";
  if (tolResult )
  {
    payload += "\"tol\":" + String(tol)+",";
  }

  if (nh4Result)
  {
    payload += "\"nh4\":" + String(nh4)+",";
  }  
  if (aceResult)
  {
    payload += "\"ace\":" + String(ace)+",";
  }
    if (co2Result)
  {
    payload += "\"co2\":" + String(co2)+",";
  }
    if (coResult)
  {
    payload += "\"co\":" + String(co)+",";
  }
    if (formResult)
  {
    payload += "\"form\":" + String(form)+",";
  }
  
    if (tempResult)
  {
    payload += "\"temp\":" + String(temp)+",";
  }
   if (humResult)
  {
    payload += "\"hum\":" + String(hum)+","+"\"mid\":"+ mid;
  }
  payload += "}";
  Serial.println(payload);
  
  mqttClient.publish("sensor/total", payload.c_str());
  delay(500);
}

bool getTolue(float *tol)//톨루엔
{ 
  if (sub_tol_dt < sub_tol)
  {
    return false;
  }
  MQ135.setA(44.947);
  MQ135.setB(-3.445); //툴루엔
  *tol = MQ135.readSensor();
  sub_tol_dt = 0;
  return true;
}
bool getNh4(float *nh4) //암모니아
{ 
  if (sub_nh4_dt < sub_nh4)
  {
    return false;
  }
  MQ135.setA(102.2);
  MQ135.setB(-2.473); //암모니아
  *nh4 = MQ135.readSensor();
  sub_nh4_dt = 0;
  return true;
}
bool getAce(float *ace) //아세톤
{ 
  if (sub_ace_dt < sub_ace)
  {
    return false;
  }
  MQ135.setA(34.668);
  MQ135.setB(-3.369); //아세톤
  *ace = MQ135.readSensor();
  sub_ace_dt = 0;
  return true;
}
bool getCo2(float *co2) //이산화탄소
{ 
  if (sub_co2_dt < sub_co2)
  {
    return false;
  }
  
  *co2 = mhz.getCO2(); //CO2가스
  sub_co2_dt = 0;
  return true;
  

}
bool getCo(float *co) //일산화탄소
{ 
  if (sub_co_dt < sub_co  )
  {
    return false;
  }
  MQ135.setA(605.18); 
  MQ135.setB(-3.937); 
  *co = MQ135.readSensor();
  sub_co_dt = 0;
  return true;
}
bool getForm(float *form) //포름알데히드
{ 
  if (sub_form_dt < sub_form)
  {
    return false;
  }
   if (hchoSensor.available() > 0)
  {
    *form  = hchoSensor.uartReadPPM();
  }
  sub_form_dt = 0;
  return true;
}
bool getTemp(float *temp) //온도
{ 
  if (sub_temp_dt < sub_temp)
  {
    return false;
  }
 
  *temp = dht.readTemperature();
  sub_temp_dt = 0;
  return true;
}
bool getHumidity(float *hum) //습도
{ 
  if (sub_hum_dt < sub_hum)
  {
    return false;
  }

  *hum = dht.readHumidity();
  sub_hum_dt = 0;
  return true;
}
