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
WebServer server(80);

LocalRecords locRec;
ChargeRecord lastRecord;
unsigned long lastConnectTime=0;

void IRAM_ATTR EM() {
  /*
  rapiSender.sendCmd("$FD");
  log_e("SHUT DOWN EVSE!!!!");
  vTaskDelay(5000);
  ESP.restart();
  */
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

  scr.bootDrawStartFrame();
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
  pinMode(BUTTON_1, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), EM, RISING);
  
  scr.bootDrawDone();
  //vTaskDelay(2000);
  // Normal mode
  scr.normalDrawFrame(eth_mac, eth_ip, setting.name);
  scr.normalDrawDeviceStatus(isAuthByCard);
  buzzer.Success();

}

void loop()
{
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
            scr.bootDrawFrame();
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
          double voltage = volts;
          if (lastVolts != voltage) {
            scr.normalDrawConcurrentVoltage(voltage);
          } 

          if (lastAmps != amps) {
            scr.normalDrawConcurrentAmp(amps);
          }
          lastVolts = voltage;
          lastAmps = amps;
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
