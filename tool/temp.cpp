#include <Adafruit_GFX.h>
#include <Adafruit_RA8875.h>
#include <U8g2_for_Adafruit_GFX.h>
#include "screen.h"
#include "fonts.h"

Screen::Screen(uint8_t cs, uint8_t rst, uint8_t sck, uint8_t miso, uint8_t mosi) : _tft(cs, rst, sck, miso, mosi)
{
    _p_cs = cs;
    _p_rst = rst;
    _p_sck = sck;
    _p_miso = miso;
    _p_mosi = mosi;
    _tft = Adafruit_RA8875(_p_cs, _p_rst, _p_sck, _p_miso, _p_mosi);
}

bool Screen::begin(RA8875sizes s)
{
    _size = s;

    _u8g2.begin(_tft);
    if (!_tft.begin(_size))
    {
        log_e("[Display::begin] RA8875 begin failed");
        return false;
    }
    _u8g2.begin(_tft);

    _tft.displayOn(true);
    _tft.GPIOX(true);
    _tft.PWM1config(true, RA8875_PWM_CLK_DIV1024);
    _tft.PWM1out(255);
    _tft.fillScreen(RA8875_BLACK);
    _tft.textMode();
    return true;
}
void Screen::twolayerMode()
{
    _tft.writeCommand(0x20);
    _tft.writeData(0x80);
}

void Screen::memorytolayer(int layer)
{
    if(layer == 1){
    _tft.writeCommand(0x41);
    // Set bit 1
    _tft.writeData(0x00);
  }
  else{
    _tft.writeCommand(0x41);
    // Set bit 1
    _tft.writeData(0x01);
  }
  delay(17);
}

void Screen::displaytolayer(int layer)
{
    if(layer == 1){
    _tft.writeCommand(0x52);
    // Set bit 1
    _tft.writeData(0x00);
  }
  else  {
    _tft.writeCommand(0x52);
    // Set bit 1
    _tft.writeData(0x01);
  }
  delay(17);
}

void Screen::Source_layer1(void)
{
    uint8_t temp;
    _tft.writeCommand(0x57);//VSBE1
     temp = _tft.readData();
     temp &= 0x80 ;
    _tft.writeData(temp);
}

void Screen::Source_layer2(void)
{
    uint8_t temp;
    _tft.writeCommand(0x57);//VSBE1
    temp = _tft.readData();
    temp |= 0x80 ;
    _tft.writeData(temp);
}

void Screen::Destination_layer1(void)
{
    uint8_t temp;
    _tft.writeCommand(0x5B);//VSBE1
     temp = _tft.readData();
     temp &= 0x80 ;
    _tft.writeData(temp);
}

void Screen::Destination_layer2(void)
{
    uint8_t temp;
    _tft.writeCommand(0x5B);//VSBE1
    temp = _tft.readData();
    temp |= 0x80 ;
    _tft.writeData(temp);
}

void Screen::BTE_Source_Destination(uint16_t XL,uint16_t XR ,uint16_t YT ,uint16_t YB)
{
    uint8_t temp,temp1;
    
    temp=XL;   
    _tft.writeCommand(0x54);//HSBE0
    _tft.writeData(temp);
    temp=XL>>8;   
    _tft.writeCommand(0x55);//HSBE1     
    _tft.writeData(temp);

    temp=XR;   
    _tft.writeCommand(0x58);//HDBE0
    _tft.writeData(temp);
    temp=XR>>8;   
    _tft.writeCommand(0x59);//HDBE1     
    _tft.writeData(temp); 
    
    temp=YT;   
    _tft.writeCommand(0x56);//VSBE0
    _tft.writeData(temp);
    temp=YT>>8;   
    _tft.writeCommand(0x57);//VSBE1
    temp1 = _tft.readData();
    temp1 &= 0x80;
    temp=temp|temp1; 
    _tft.writeCommand(0x57);//VSBE1  
    _tft.writeData(temp);


    temp=YB;   
    _tft.writeCommand(0x5a);//VDBE0
    _tft.writeData(temp);
    temp=YB>>8;   
    _tft.writeCommand(0x5b);//VDBE1
    temp1 = _tft.readData();
    temp1 &= 0x80;
    temp=temp|temp1;  
    _tft.writeCommand(0x5b);//VDBE1   
    _tft.writeData(temp);
}

void Screen:: BTE_Size_setting(uint16_t X,uint16_t Y)
{  
    _tft.writeCommand(0x5c);//BEWR0
  _tft.writeData(X); 
    _tft.writeCommand(0x5d);//BEWR1     
  _tft.writeData(X>>8);
  
    _tft.writeCommand(0x5e);//BEHR0
  _tft.writeData(Y);   
    _tft.writeCommand(0x5f);//BEHR1     
  _tft.writeData(Y>>8);
}                  


void Screen::BTE_ROP_Code(uint8_t setx)
{  
    _tft.writeCommand(0x51);//BECR1     
    _tft.writeData(setx); 
}                    

void Screen:: Chk_Busy_BTE(void)
{
  uint8_t temp;
    
  do
  {
     temp=_tft.readStatus();
  }while((temp&0x40)==0x40);     
}


void Screen::btememorycopy(uint8_t source_layer,uint16_t s_x,uint16_t s_y,
                                      uint8_t destination_layer,uint16_t des_x,uint16_t des_y,
                                      uint16_t block_width,uint16_t block_height)
{
  if(source_layer==1)
  Source_layer1();
  else if (source_layer==2)
  Source_layer2();

  if(destination_layer==1)
  Destination_layer1();
  else if (destination_layer==2)
  Destination_layer2();

  BTE_Source_Destination(s_x,des_x,s_y,des_y);
  BTE_Size_setting(block_width,block_height); 
  BTE_ROP_Code(0xc2); //(Move BTE in positive direction with ROP.)
   
  _tft.writeCommand(0x50); //BTE enable
  _tft.writeData(0x80);
  Chk_Busy_BTE();

}

void Screen::bootDrawFrame()
{
    // 開機中框架畫面
    memorytolayer(2);
    _tft.fillScreen(RA8875_BLACK);
    _tft.fillRect(0, 0, 480, 48, RA8875_GREEN);
    _u8g2.setFontMode(2);
    _u8g2.setFontDirection(0);
    _u8g2.setForegroundColor(RA8875_BLUE);
    _u8g2.setBackgroundColor(RA8875_GREEN);
    _u8g2.setFont(cht_font_32);
    _u8g2.setCursor(0, 40);
    _u8g2.print("EVSE智慧型充電樁");

    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setFont(cht_font_32);
    _u8g2.setCursor(100, 120);
    _u8g2.print("開機中....");
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::bootDrawStatu(String status)
{
    memorytolayer(2);
    _tft.fillRect(100, 130, 480, 160, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setCursor(100, 160);
    _u8g2.setFont(cht_font_24);
    _u8g2.print("目前狀態 => " + status);
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::bootDrawError(String error)
{
    memorytolayer(2);
    // 開機中框架畫面
    _tft.fillScreen(RA8875_BLACK);
    _tft.fillRect(0, 0, 480, 48, RA8875_GREEN);
    _u8g2.setFontMode(2);
    _u8g2.setFontDirection(0);
    _u8g2.setForegroundColor(RA8875_BLUE);
    _u8g2.setBackgroundColor(RA8875_GREEN);
    _u8g2.setFont(cht_font_32);
    _u8g2.setCursor(0, 40);
    _u8g2.print("EVSE智慧型充電樁");

    _tft.fillRect(100, 120, 480, 160, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_RED);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setCursor(100, 160);
    _u8g2.setFont(cht_font_24);
    _u8g2.print("偵測到錯誤 => " + error);
    _u8g2.setCursor(100, 200);
    _u8g2.setFont(cht_font_24);
    _u8g2.print("請嘗試重新開機，或聯繫管理員");
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::bootDrawDone()
{
    memorytolayer(2);
    _tft.fillRect(100, 120, 480, 272, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setFont(cht_font_32);
    _u8g2.setCursor(100, 120);
    _u8g2.print("開機完成，進入正常模式");
    _u8g2.setCursor(100, 160);
    _u8g2.print("請稍後....");
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::normalDrawFrame(String mac, String ip, String position)
{
    memorytolayer(2);
    // Color Block
    _tft.fillRect(0, 48, 240, 192, RA8875_WHITE);
    _tft.fillRect(240, 48, 240, 192, RA8875_BLACK);
    _tft.fillRect(0, 240, 480, 32, RA8875_GREEN);
    // IP & Mac
    _u8g2.setForegroundColor(RA8875_BLUE);
    _u8g2.setBackgroundColor(RA8875_GREEN);
    _u8g2.setFont(cht_font_16);
    _u8g2.setCursor(330, 20);
    _u8g2.print(mac);
    _u8g2.setForegroundColor(RA8875_BLUE);
    _u8g2.setBackgroundColor(RA8875_GREEN);
    _u8g2.setFont(cht_font_16);
    _u8g2.setCursor(330, 40);
    _u8g2.print(ip);
    // Position
    _u8g2.setForegroundColor(RA8875_BLACK);
    _u8g2.setBackgroundColor(RA8875_GREEN);
    _u8g2.setFont(cht_font_24);
    _u8g2.setCursor(250, 264);
    _u8g2.print("車位：" + position);
    btememorycopy(2,0,0,1,0,0,480,272);
}
void Screen::normalDrawConcurrentVoltage(double voltage)
{
    memorytolayer(2);
    _tft.fillRect(310, 48, 170, 24, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setFont(cht_font_24);
    _u8g2.setCursor(250, 72);
    _u8g2.print("電壓: " + String(voltage) + " V");
    btememorycopy(2,0,0,1,0,0,480,272);
}
void Screen::normalDrawConcurrentAmp(double amp)
{
    memorytolayer(2);
    _tft.fillRect(310, 72, 170, 24, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setFont(cht_font_24);
    _u8g2.setCursor(250, 96);
    _u8g2.print("電流: " + String(amp) + " A");
    btememorycopy(2,0,0,1,0,0,480,272);
}
void Screen::normalDrawConcurrentWatts(double watts)
{
    memorytolayer(2);
    _tft.fillRect(310, 96, 170, 24, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setFont(cht_font_24);
    _u8g2.setCursor(250, 120);
    _u8g2.print("充電度數: " + String(watts) + " kwh");
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::normalDrawTotalWatts(double watts)
{
    memorytolayer(2);
    _tft.fillRect(310, 120, 170, 24, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setFont(cht_font_24);
    _u8g2.setCursor(250, 148);
    _u8g2.print("總度數: " + String(watts) + " kwh");
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::normalDrawTemp(double temp) {
    memorytolayer(2);
    _tft.fillRect(310, 148, 170, 24, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setFont(cht_font_24);
    _u8g2.setCursor(250, 172);
    _u8g2.print("目前溫度: " + String(temp));
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::normalDrawPlugStatus(String status)
{
    memorytolayer(2);
    // Position
    _tft.fillRect(0, 240, 240, 32, RA8875_GREEN);
    _u8g2.setForegroundColor(RA8875_BLACK);
    _u8g2.setBackgroundColor(RA8875_GREEN);
    _u8g2.setFont(cht_font_16);
    _u8g2.setCursor(10, 264);
    _u8g2.print("狀態: " + status);
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::normalDrawDateTime()
{
    memorytolayer(2);
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return;
    }
    char buf[20];
    _tft.fillRect(310, 188, 170, 48, RA8875_BLACK);
    _u8g2.setForegroundColor(RA8875_WHITE);
    _u8g2.setBackgroundColor(RA8875_BLACK);
    _u8g2.setFont(cht_font_24);

    strftime(buf, 20, "%Y/%m/%d", &timeinfo);
    _u8g2.setCursor(250, 212);
    _u8g2.print(String(buf));

    strftime(buf, 20, "%H:%M:%S", &timeinfo);
    _u8g2.setCursor(250, 236);
    _u8g2.print(String(buf));
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::normalDrawDeviceStatus(bool isAuth) {
    memorytolayer(2);
    if (isAuth) {
        _tft.fillRect(0, 48, 240, 112, RA8875_WHITE);
        _u8g2.setForegroundColor(RA8875_BLUE);
        _u8g2.setBackgroundColor(RA8875_WHITE);
        _u8g2.setFont(cht_font_32);
        _u8g2.setCursor(10, 92);
        _u8g2.print("設備開放中");
        // _tft.fillRect(0, 96, 240, 48, RA8875_WHITE);
        // _u8g2.setForegroundColor(RA8875_BLACK);
        // _u8g2.setBackgroundColor(RA8875_WHITE);
        // _u8g2.setFont(cht_font_24);
        // _u8g2.setCursor(10, 132);
        // _u8g2.print("鎖定倒數: " + String(count) + " 秒");
    } 
    else 
    {
        _tft.fillRect(0, 48, 240, 64, RA8875_WHITE);
        _u8g2.setForegroundColor(RA8875_RED);
        _u8g2.setBackgroundColor(RA8875_WHITE);
        _u8g2.setFont(cht_font_32);
        _u8g2.setCursor(10, 92);
        _u8g2.print("設備鎖定中");
        _tft.fillRect(0, 96, 240, 48, RA8875_WHITE);
        _u8g2.setForegroundColor(RA8875_BLACK);
        _u8g2.setBackgroundColor(RA8875_WHITE);
        _u8g2.setFont(cht_font_24);
        _u8g2.setCursor(10, 132);
        _u8g2.print("請刷卡解鎖");
    }
    btememorycopy(2,0,0,1,0,0,480,272);
}

void Screen::normalDrawDeviceWait() {
    memorytolayer(2);
    _tft.fillRect(0, 48, 240, 64, RA8875_WHITE);
    _u8g2.setForegroundColor(RA8875_RED);
    _u8g2.setBackgroundColor(RA8875_WHITE);
    _u8g2.setFont(cht_font_32);
    _u8g2.setCursor(10, 92);
    _u8g2.print("電樁暫時無法使用");
    _tft.fillRect(0, 96, 240, 48, RA8875_WHITE);
    _u8g2.setForegroundColor(RA8875_BLACK);
    _u8g2.setBackgroundColor(RA8875_WHITE);
    _u8g2.setFont(cht_font_24);
    _u8g2.setCursor(10, 132);
    _u8g2.print("等待更新紀錄");
    btememorycopy(2,0,0,1,0,0,480,272);
}


void Screen::normalDrawDeviceMessage(String message, bool isError) {
    memorytolayer(2);

    btememorycopy(2,0,0,1,0,0,480,272);
}


void Screen::normalDrawMenu(uint16_t key){
    memorytolayer(2);
    switch (key)
    {
    case 0:
        _tft.fillRect(0, 48 , 480, 192, RA8875_BLACK);
        _u8g2.setForegroundColor(RA8875_WHITE);
        _u8g2.setBackgroundColor(RA8875_BLACK);
        _u8g2.setCursor(0, 74);
        _u8g2.setFont(cht_font_24);
        _u8g2.print("1.直接充電\n2.時間電價充電\n3.排程充電\n4.XXXXXX\n5.OOOOOOO");
        break;

    case 0x0001:
        _tft.fillRect(0, 48 , 240, 24, RA8875_WHITE);
        _u8g2.setForegroundColor(RA8875_BLACK);
        _u8g2.setBackgroundColor(RA8875_WHITE);
        _u8g2.setCursor(0, 74);
        _u8g2.setFont(cht_font_24);
        _u8g2.print("1.直接充電");
        break;
    
    case 0x0002:
        _tft.fillRect(0, 72 , 240, 24, RA8875_WHITE);
        _u8g2.setForegroundColor(RA8875_BLACK);
        _u8g2.setBackgroundColor(RA8875_WHITE);
        _u8g2.setCursor(0, 74);
        _u8g2.setFont(cht_font_24);
        _u8g2.print("\n2.時間電價充電");
        break;

    case 0x0004:
        _tft.fillRect(0, 96 , 240, 24, RA8875_WHITE);
        _u8g2.setForegroundColor(RA8875_BLACK);
        _u8g2.setBackgroundColor(RA8875_WHITE);
        _u8g2.setCursor(0, 74);
        _u8g2.setFont(cht_font_24);
        _u8g2.print("\n\n3.排程充電");
        break;

    case 0xFFFF:
        _tft.fillRect(0, 48 , 480, 192, RA8875_BLACK);
        _tft.fillRect(0, 48 , 240, 192, RA8875_WHITE);
    
    default:
        break;
    }
    

    btememorycopy(2,0,0,1,0,0,480,272);
}

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <openevse.h>
#include <Adafruit_GFX.h>
#include <Adafruit_RA8875.h>
#include <SoftwareSerial.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <WebServer_ESP32_SC_W5500.h>
#include <uri/UriBraces.h>
#include <uri/UriRegex.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Adafruit_MCP9808.h>
#include <ZMPT101B.h>
// 自己管的部分
#include <Wiegand.h>
#include <MFRC522_I2C.h>
#include <buzzer.h>
#include <screen.h>
#include <define.h>
#include <utils.h>
#include <records.h>

// Controller Version
String controllerVersion = "1.0.0-20131205";
// Controller Version

RapiSender rapiSender(&RAPI_PORT);
Screen scr = Screen(LCD_CS, LCD_RST, LCD_SCK, LCD_MISO, LCD_MOSI);
Adafruit_MCP9808 tempSensor;
Setting setting;
String eth_ip, eth_mac;
Buzzer buzzer = Buzzer(BEEPER,PWMCHANNEL,RESOLUTION);
unsigned long lock_countdown=0;
uint32_t next_status = 0;

WIEGAND wg;

double lastVolts=-1, lastAmps=-1, lastWalts=-1, maxAmps = -1;
double lastSessionWatts = -1, lastTotalWatts = -1;

unsigned long epochValidUntil=0;

bool isCharging = false, lastIsCharging = false;
bool isConnected = false, lastIsConnected = false;
bool isConnectEMS = false;
String lastAuthenCard = "";
String evseVersion = "", evseProtocol=""; 
bool isLocked = true;
bool isAuthByCard = false;
bool isemergency = false;
WebServer server(80);

LocalRecords locRec;
ChargeRecord lastRecord;
unsigned long lastConnectTime=0;

void IRAM_ATTR EM() {
  
  isemergency = true;
  
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(ATMEGA32_BAUD, SERIAL_8N1, ATMEGA32_RX, ATMEGA32_TX);
  while (!Serial && millis() < 5000);
    delay(500);
  scr.begin(RA8875_480x272);

  //rapiSender.sendCmd("$FD");
  rapiSender.sendCmd("$S4 1");
  rapiSender.sendCmd("$SV 220000");

  scr.bootDrawFrame();
  scr.bootDrawStatu("設定腳位");
  //vTaskDelay(1000);
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED3,HIGH);

  scr.bootDrawStatu("設定揚聲器");
  //vTaskDelay(1000);
  buzzer.begin();
  buzzer.Success();


  int count = 0;
  scr.bootDrawStatu("設定序列通訊");
  //vTaskDelay(1000);
  Wire.begin(RFID_SDA, RFID_SCL);
  log_e("[Setup] Starting I2C scan:");
  for (byte i = 8; i < 128; i++)
  {
    Wire.beginTransmission(i);       // Begin I2C transmission Address (i)
    byte error = Wire.endTransmission();
    if (error == 0) // Receive 0 = success (ACK response)
    {
      log_e("[Setup] \tFound address: %d", i);
      count++; 
    }
  }
  log_e("[Setup] \tFound %d devices(s).", count);
  
  scr.bootDrawStatu("設定RFID");
  //vTaskDelay(1000);
  wg.begin(19,20);

  scr.bootDrawStatu("設定溫度傳感器");
  //vTaskDelay(1000);
  tempSensor.begin();

  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  
  
  scr.bootDrawStatu("起始緊急開關綁定");
  //vTaskDelay(1000);
  pinMode(BUTTON_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), EM, FALLING);
  if(digitalRead(BUTTON_1)==LOW)
  {
    scr.bootDrawError("緊急開關未復位");
    while(true)
    {
      delay(100);
    }
  }
  scr.bootDrawDone();
  //vTaskDelay(2000);
  // Normal mode
  scr.normalDrawFrame(eth_mac, eth_ip, setting.name);
  scr.normalDrawDeviceStatus(isAuthByCard);
  buzzer.Success();

}

void loop()
{
  if(isemergency==true){
    rapiSender.sendCmd("$FD");
    log_e("SHUT DOWN EVSE!!!!");
    scr.bootDrawError("緊急狀況 裝置鎖定");
    while(true)
    {
      delay(10);
    }
    delay(5000);
    ESP.restart();
  }
  

  rapiSender.loop();

  if(millis() > next_status)
  {
    next_status = millis() + POLL_TIME;
    if(OpenEVSE.isConnected())
    {

      OpenEVSE.getStatus([](int ret, uint8_t evse_state, uint32_t session_time, uint8_t pilot_state, uint32_t vflags)
      {
        if(RAPI_RESPONSE_OK == ret)
        {
          if(evse_state==OPENEVSE_STATE_CHARGING){
            digitalWrite(LED2,HIGH);
          }
          else
          {
            digitalWrite(LED2,LOW);
          }
          String state_msg;
          bool failed = false;
          isCharging = false;
          switch (evse_state) {
            case OPENEVSE_STATE_STARTING:
              state_msg = "開機中...."; break;
            case OPENEVSE_STATE_NOT_CONNECTED:
              state_msg = "車輛未連接"; 
              isConnected = false;
              break;
            case OPENEVSE_STATE_CONNECTED:
              state_msg = "車輛已連接"; 
              isConnected = true;
              break;
            case OPENEVSE_STATE_CHARGING:
              state_msg = "車輛充電中"; 
              isCharging = true;
              break;
            case OPENEVSE_STATE_VENT_REQUIRED:
              state_msg = "車輛需要放電"; break;
            case OPENEVSE_STATE_DIODE_CHECK_FAILED:
              state_msg = "Diode檢查失敗";
              failed = true;
              break;
            case OPENEVSE_STATE_GFI_FAULT:
              state_msg = "GFI檢查失敗";
              failed = true;
              break;
            case OPENEVSE_STATE_NO_EARTH_GROUND:
              state_msg = "接地檢查失敗";
              failed = true;
              break;
            case OPENEVSE_STATE_STUCK_RELAY:
              state_msg = "繼電器阻塞";
              failed = true;
              break;
            case OPENEVSE_STATE_GFI_SELF_TEST_FAILED:
              state_msg = "GFI自檢失敗";
              failed = true;
              break;
            case OPENEVSE_STATE_OVER_TEMPERATURE:
              state_msg = "溫度過高";
              failed = true;
              break;
            case OPENEVSE_STATE_OVER_CURRENT:
              state_msg = "電流過高";
              failed = true;
              break;
            case OPENEVSE_STATE_SLEEPING:
              state_msg = "睡眠中"; break;
            case OPENEVSE_STATE_DISABLED:
              state_msg = "設備已鎖定"; break;
            default:
              state_msg = "不明的狀態";
              failed = true;
              break;
          }
          if (failed) {
            log_e("發生錯誤");
            rapiSender.sendCmd("$FD");
            scr.bootDrawError(state_msg);
            while (true)
              delay(10);
          } else {
            
            scr.normalDrawPlugStatus(state_msg);

          }
        }
      });
      OpenEVSE.getChargeCurrentAndVoltage([](int ret, double amps, double volts) {
        if (RAPI_RESPONSE_OK == ret) {
          
          if (lastVolts != volts) {
            scr.normalDrawConcurrentVoltage(volts);
          } 

          if (lastAmps != amps) {
            scr.normalDrawConcurrentAmp(amps);
          }
          lastVolts = volts;
          lastAmps = amps;
          String data_output =String(volts)+","+String(amps);
          Serial.println(data_output);
        }
      });
      OpenEVSE.getEnergy([](int ret, double session_wh, double total_kwh) {
        if (RAPI_RESPONSE_OK == ret) {
          if (lastSessionWatts != session_wh) {
            scr.normalDrawConcurrentWatts(session_wh);
          }

          /*
          if (lastTotalWatts != total_kwh) {
            scr.normalDrawTotalWatts(total_kwh);
          }
          */

          lastSessionWatts = session_wh;
          lastTotalWatts = total_kwh;
        }
      }); 
      if (evseVersion == "") {
        OpenEVSE.getVersion([](int ret, const char* firmware, const char* protocol) {
          if (RAPI_RESPONSE_OK == ret) {
            evseVersion = String(firmware);
            evseProtocol = String(protocol);
          }
        });
      }
       /* 充電電流判定 */
      OpenEVSE.getCurrentCapacity([](int ret, long min_current, long pilot, long max_configured_current, long max_hardware_current){
        if (RAPI_RESPONSE_OK == ret) {
          
            maxAmps = max_configured_current;
           
          }
      });
    }
    else
    {

      OpenEVSE.begin(rapiSender, [](bool connected)
      {
        if(connected)
        {
          log_e("Connected to OpenEVSE\n");
        } else {
          log_e("OpenEVSE not responding or not connected");
        }
      });
    }
  }

  // 溫度
  sensors_event_t event;
  tempSensor.getEvent(&event);
  scr.normalDrawTemp(event.temperature);
  //scr.normalDrawDateTime();

  // if (voltageSensor1.getRmsVoltage()<150)
  // {
  //   voltageSensor1.setSensitivity(578);
  //   lastVolts = voltageSensor1.getRmsVoltage();
  // }
  // else
  // {
  //   voltageSensor1.setSensitivity(658);
  //   lastVolts = voltageSensor1.getRmsVoltage();
  // }
  // scr.normalDrawConcurrentVoltage(lastVolts);
  //scr.normalDrawDateTime();


  if (lastIsCharging == true && isCharging == false) {
    // 從充電 -> 不充電 ==> 充電結束鎖定
    log_e("From CHARGING to NOTCHARGING");
  } else if (lastIsCharging == false && isCharging == true) {
    // 從非充電 -> 充電 ==> 充電起始
    log_e("From NOTCHARGING to CHARGING");
  } else {
    //log_e("CHARGING Status= %s",isCharging? "true": "false" );
  }
  lastIsCharging = isCharging;

  //連接到電動車
  if (lastIsConnected == true && isConnected == false) {
    log_e("Plug From CONNECTED to DISCONNECTED");
    isAuthByCard = false;
    scr.normalDrawDeviceStatus(isAuthByCard);
    lastAuthenCard = "";
    rapiSender.sendCmd("$S4 1");
  } else if (lastIsConnected == false && isConnected == true) {
    log_e("Plug From DISCONNECT to CONNECTED");
  } else {
    //log_e("Plug Status= %s", isConnected? "true": "false" );
  }
  lastIsConnected = isConnected;

  // while(isAuthByCard&&!ExitMenuMode){
  //   uint16_t keyinput = 0x0000;
  //   log_e("Start Meun Mode");
  //   scr.normalDrawMenu(0);
  //   //wait keypad input
  //   while(keyinput==0x0000)
  //   {
  //     keyinput = keyscan();
  //     delay(50);
  //   }
  //   scr.normalDrawMenu(keyinput);
  //   log_e("KEYPAD INPUT IS : %d",keyinput);
  //   delay(1000);
  //   scr.normalDrawMenu(EXIT_MEUNMODE);
  //   ExitMenuMode = true;
  //   meuntime = millis();

  // }

  if ( !wg.available() || !isConnected) {
    delay(50);
    return;
  }

  String card_uuid = String(wg.getCode(),HEX);
  log_e("Card ID: %s", card_uuid);
  bool checked = false;
  for (int x=0; x<10; x++) {
    if (setting.validTag[x].length()==0) break;
    if (setting.validTag[x] == card_uuid) {
      checked = true;
    }
  }
  checked = true;
  if (checked) {
    log_e("Card ID: %s is valid", card_uuid);
    buzzer.Success();
    // 跟上一張一樣卡號的話, 鎖上
    if (card_uuid == lastAuthenCard) {
      log_e("From UNLOCK to LOCK");
      isAuthByCard = false;
      scr.normalDrawDeviceStatus(isAuthByCard);
      rapiSender.sendCmd("$S4 1"); 
      lastAuthenCard = "";
    } else {
      log_e("From LOCK to UNLOCK");
      isAuthByCard = true;
      scr.normalDrawDeviceStatus(isAuthByCard);
      rapiSender.sendCmd("$S4 0");
      lastAuthenCard = card_uuid;
    }
  } else {
    log_e("Card ID: %s is invalid", card_uuid);
    buzzer.Fail();
    epochValidUntil=0;
  }

  vTaskDelay(1000);
}

