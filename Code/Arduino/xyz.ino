#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)    //可修改

//引脚定义
#define BC26_RESET_PIN        A0
#define BC26_PWR_KEY          6
#define BC26_PSM_PIN          7
#define FLA_sensorPin         A1
#define FLA_intPin            3

#define DEFAULT_TIMEOUT       10   //seconds
#define BUF_LEN               300

//BC26输出指令定义
#define BC26_SUCC_RSP               "OK"
#define BC26_AT_CHECK               "AT\r\n"
#define BC26_ECHO_OFF               "ATE0\r\n"
#define BC26_CHECK_GPRS_ATTACH      "AT+CGATT?\r\n"
#define BC26_CGATT_SUCC_POSTFIX     "+CGATT: 1"
#define BC26_ALI_PRESET             "AT+QMTCFG=\"ALIAUTH\",0,\"%s\",\"%s\",\"%s\"\r\n"
#define BC26_ALI_OPEN_LINK          "AT+QMTOPEN=0,\"iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883\r\n"
#define BC26_ALI_OPEN_LINK_SUCC     "+QMTOPEN: 0,0"
#define BC26_ALI_BUILD_LINK         "AT+QMTCONN=0,\"clientExample\"\r\n"
#define BC26_ALI_BUILD_LINK_SUCC    "+QMTCONN: 0,0,0"
#define BC26_ALI_SENSOR_UPLOAD      "AT+QMTPUB=0,12,1,0,\"/sys/%s/%s/thing/event/property/post\",\"{'id':'110','version':'1.0','method':'thing.event.property.post','params':{'Temperature':%d.%02d,'Humidity':%d.%02d,'Flame':%d}}\"\r\n"
#define BC26_ALI_SENSOR_UPLOAD_SUCC "+QMTPUB: 0,12,0"
#define BC26_ALI_DISCON             "AT+QMTDISC=0\r\n"
#define BC26_ALI_DISCON_SUCC        "+QMTDISC: 0,0"

//设备密钥定义 
#define ProductKey                  "Secrets_Not_Shown"
#define DeviceName                  "Sensor"
#define DeviceSecret                "Secrets_Not_Shown"

//变量定义
char      ATcmd[BUF_LEN];
char      ATbuffer[BUF_LEN];
unsigned  bme280_status;
float     bme280_temperature;
float     bme280_humidity;
int       FLA_sensorValue;
Adafruit_BME280       bme;

void setup() {
  Serial.begin(115200);
  BC26_init();
  pinMode(FLA_sensorPin,INPUT);
  pinMode(FLA_intPin,INPUT);
  bme280_status = bme.begin();
  while(!bme280_status) bme280_status = bme.begin();
}

void loop() {
  while(1)
  {
    BC26_reset();    
    if(!BC26_network_check())continue;
    if(!BC26_Ali_connect())continue;
    break;
  }
  for(int count = 1;1;count++)          //持续读取数据
  {
    int FLA_sensorValue = analogRead(FLA_sensorPin);
    bme280_temperature = bme.readTemperature();
    bme280_humidity = bme.readHumidity();
    BC26_sensor_upload(bme280_temperature,bme280_humidity,FLA_sensorValue);    
    delay(100);
  }
  while(!check_send_cmd(BC26_ALI_DISCON,BC26_ALI_DISCON_SUCC,DEFAULT_TIMEOUT));
  while(1);
}



//BC26传输相关函数

//BC26初始化
void BC26_init()
{
  pinMode(BC26_RESET_PIN,OUTPUT);
  pinMode(BC26_PWR_KEY,OUTPUT);
  pinMode(BC26_PSM_PIN,OUTPUT);
  digitalWrite(BC26_PWR_KEY,LOW);
  digitalWrite(BC26_PSM_PIN,LOW);
}
//BC26重置
void BC26_reset()
{
  digitalWrite(BC26_RESET_PIN,HIGH);
  delay(100);
  digitalWrite(BC26_RESET_PIN,LOW);
  delay(20000);
  Serial.flush();
}
//网络检查，成功返回1
bool BC26_network_check()
{
  bool flag;
  
  flag = check_send_cmd(BC26_AT_CHECK,BC26_SUCC_RSP,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(BC26_ECHO_OFF,BC26_SUCC_RSP,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(BC26_CHECK_GPRS_ATTACH,BC26_CGATT_SUCC_POSTFIX,DEFAULT_TIMEOUT);
  return flag;
}
//ALI云连接，成功返回1
bool BC26_Ali_connect()
{
  bool flag;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,BC26_ALI_PRESET,ProductKey,DeviceName,DeviceSecret);
  flag = check_send_cmd(ATcmd,BC26_SUCC_RSP,DEFAULT_TIMEOUT);
  if(!flag)return false;
     
  for (int count = 1;count <= 3;count++)
  {
    flag = check_send_cmd(BC26_ALI_OPEN_LINK,BC26_ALI_OPEN_LINK_SUCC,40);
    if(flag)break;
  }
  if(!flag)return false;

  flag = check_send_cmd(BC26_ALI_BUILD_LINK,BC26_ALI_BUILD_LINK_SUCC,DEFAULT_TIMEOUT);
  return flag;
}
//传感器参数上传，成功返回1
bool BC26_sensor_upload(float temperature,float humidity,int sensorValue)
{
  bool flag;
  int inte1,frac1;
  int inte2,frac2;
  int inte3;
  
  cleanBuffer(ATcmd,BUF_LEN);
  
  inte1 = temperature;
  frac1 = (temperature - inte1) * 100;
  inte2 = humidity;
  frac2 = (humidity - inte2) * 100;
  inte3 = sensorValue;


  snprintf(ATcmd,BUF_LEN,BC26_ALI_SENSOR_UPLOAD,ProductKey,DeviceName,inte1,frac1,inte2,frac2,inte3);
  flag = check_send_cmd(ATcmd,BC26_ALI_SENSOR_UPLOAD_SUCC,20);
  return flag;
}
//传输核查，成功则返回1
bool check_send_cmd(const char* cmd,const char* resp,unsigned int timeout)
{
  int i = 0;
  unsigned long timeStart;
  timeStart = millis();
  cleanBuffer(ATbuffer,BUF_LEN);
  Serial.print(cmd);
  
  while(1)
  {
    while(Serial.available())
    {
      ATbuffer[i++] = Serial.read();
      if(i >= BUF_LEN)return false;
    }
    if(NULL != strstr(ATbuffer,resp))break;
    if((unsigned long)(millis() - timeStart > timeout * 1000)) break;
  }
  Serial.flush();
  
  if(NULL != strstr(ATbuffer,resp))return true;
  return false;
}

void cleanBuffer(char *buf,int len)
{
  for(int i = 0;i < len;i++)
  {
    buf[i] = '\0';
  }
}
