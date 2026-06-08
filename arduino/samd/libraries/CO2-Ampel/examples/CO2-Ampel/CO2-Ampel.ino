/*
  CO2-Ampel
    https://learn.watterott.com/breakouts/co2-ampel/

  Serial output
    9600 Baud 8N1

  Serial commands
    R=0      - Remote control off
    R=1      - Remote control on
    R=R      - Reset
    V?       - Query firmware version
    S=1      - Save settings
    L=RRGGBB - LED color (000000-FFFFFF)
    H=X      - LED brightness (0-FF)
    B=0      - Buzzer disabled
    B=1      - Buzzer enabled and on for 500ms
    T=X      - Temperature offset in deg C (0-20)
    T?       - Query temperature offset
    A=X      - Altitude above sea level (0-3000)
    A?       - Query altitude
    C=1      - Calibration to 400ppm (requires at least 2min running on fresh air before command)
    1=X      - Range 1 start (400-10000) - green
    2=X      - Range 2 start (400-10000) - yellow
    3=X      - Range 3 start (400-10000) - red
    4=X      - Range 4 start (400-10000) - red blinking
    5=X      - Range 5 start (400-10000) - red + buzzer
*/

#define VERSION "26"

#define COVID      0 //1 = COVID CO2 values
#define WIFI_AMPEL 0 //1 = Version with WiFi/WLAN
#define PRO_AMPEL  0 //1 = Pro version with pressure sensor

//--- CO2 values ---
#if COVID
//Covid prevention: https://www.umwelt-campus.de/forschung/projekte/iot-werkstatt/ideen-zur-corona-krise
  #define START_GREEN        600 //>= 600ppm
  #define START_YELLOW       800 //>= 800ppm
  #define START_RED         1000 //>=1000ppm
  #define START_RED_BLINK   1200 //>=1200ppm
  #define START_BUZZER      1400 //>=1400ppm
#else
//Fatigue
  #define START_GREEN        600 //>= 600ppm
  #define START_YELLOW      1000 //>=1000ppm
  #define START_RED         1200 //>=1200ppm
  #define START_RED_BLINK   1400 //>=1400ppm
  #define START_BUZZER      1600 //>=1600ppm
#endif

//--- WiFi/WLAN ---
#define WIFI_SSID          "" //WiFi SSID
#define WIFI_CODE          "" //WiFi password
#define WIFI_NM            255,255,255,  0 //Netmask
#define WIFI_IP              0,  0,  0,  0 //Local IP address, 0=DHCP
#define WIFI_GW            192,168,  1,100 //Gateway IP address
#define WIFI_DNS           192,168,  1,100 //DNS IP address

//--- Ampel brightness (LEDs) ---
#define BRIGHTNESS         180 //1-255 (255=100%, 179=70%)
#define BRIGHTNESS_DARK    20  //1-255 (255=100%, 25=10%)
#define NUM_LEDS           4   //Number of LEDs

//--- Light sensor ---
#define LIGHT_DARK         20   //<20 -> dark
#define LIGHT_INTERVAL     60 //10-120min (sensor check)

//--- General ---
#define INTERVAL           2 //2-1800s measurement interval (SCD30 only, SCD4X always 5s)
#define AMPEL_AVERAGE      1 //1 = use CO2 average for ampel
#define AUTO_CALIBRATION   0 //1 = automatic calibration (ASC) on (requires 7 days of continuous operation with 1h of fresh air per day)
#define BUZZER             1 //enable buzzer
#define BUZZER_DELAY     300 //300s, buzzer startup delay
#define TEMP_OFFSET        4 //Temperature offset in deg C (0-20)
#define TEMP_OFFSET_WIFI   8 //WiFi, temperature offset in deg C (0-20)
#define TEMP_OFFSET_PRO    6 //Pro WiFi, temperature offset in deg C (0-20)
#define PRESSURE_DIFF      5 //Pressure difference in hPa (5-20)
#define BAUDRATE           9600 //9600 Baud
#define START_VALUE        500 //500ppm, initial CO2 value

//--- Colors ---
#define COLOR_BLUE         0x007CB0 //0x0000FF, sky blue: 0x007CB0
#define COLOR_GREEN        0x00FF00 //0x00FF00
#define COLOR_YELLOW       0xFF7F00 //0xFF7F00
#define COLOR_RED          0xFF0000 //0xFF0000
#define COLOR_VIOLET       0xFF00FF //0xFF00FF
#define COLOR_WHITE        0xFFFFFF //0xFFFFFF
#define COLOR_OFF          0x000000 //0x000000

//--- I2C/Wire ---
#define ADDR_SSD1306       0x3D //0x3D or 0x3C, Wire=SERCOM0
#define ADDR_SCD30         0x61 //0x61, Wire=SERCOM0
#define ADDR_SCD4X         0x62 //0x62, Wire=SERCOM0
#define ADDR_LPS22HB       0x5C //0x5C, Wire1=SERCOM2
#define ADDR_BMP280        0x76 //0x76 or 0x77, Wire1=SERCOM2
#define ADDR_ATECC608      0x60 //0x60, Wire1=SERCOM2 (optional)

//--- Features ---
enum Features
{
  FEATURE_USB      = (1<<0),
  FEATURE_SCD30    = (1<<1),
  FEATURE_SCD4X    = (1<<2),
  FEATURE_LPS22HB  = (1<<3),
  FEATURE_BMP280   = (1<<4),
  FEATURE_WINC1500 = (1<<5),
  FEATURE_SSD1306  = (1<<6),
};


#include <Wire.h>
#include <SPI.h>
#include <FlashStorage.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_BMP280.h>
#include <Arduino_LPS22HB.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi101.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


extern USBDeviceClass USBDevice; //USBCore.cpp


typedef struct
{
  boolean valid;
  unsigned int brightness;
  unsigned int range[5];
  unsigned int buzzer;
  char wifi_ssid[64+1];
  char wifi_code[64+1];
  IPAddress netmask;
  IPAddress ip_local;
  IPAddress ip_gw;
  IPAddress ip_dns;
} SETTINGS;

SETTINGS settings;
FlashStorage(flash_settings, SETTINGS);
SCD30 scd30;
SensirionI2CScd4x scd4x;
Adafruit_BMP280 bmp280(&Wire1);
LPS22HBClass lps22(Wire1);
Adafruit_NeoPixel ws2812 = Adafruit_NeoPixel(NUM_LEDS, PIN_WS2812, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(128, 64); //128x64 pixels
WiFiServer server(80); //Webserver port 80

unsigned int features=0, remote_on=0, buzzer_timer=BUZZER_DELAY;
unsigned int co2_value=START_VALUE, co2_average=START_VALUE, light_value=1024;
float temp_value=20, temp_offset=TEMP_OFFSET, humi_value=50, pres_value=1013, pres_last=1013, temp2_value=20;


void leds(uint32_t color)
{
  ws2812.fill(color, 0, NUM_LEDS);
  ws2812.show();
}


void status_led(unsigned int on)
{
  if(on == 0)
  {
    digitalWrite(PIN_LED, LOW); //status LED off
  }
  else if(on == 1)
  {
    digitalWrite(PIN_LED, HIGH); //status LED on
  }
  else if(on < 2000)
  {
    on = on/2;
    digitalWrite(PIN_LED, HIGH); //status LED on
    delay(on); //wait ms
    digitalWrite(PIN_LED, LOW); //status LED off
    delay(on); //wait ms
  }

  return;
}


void buzzer(unsigned int on)
{
  if(on == 0)
  {
    analogWrite(PIN_BUZZER, 0); //buzzer off
  }
  else if(on == 1)
  {
    analogWrite(PIN_BUZZER, 255/2); //buzzer on
  }
  else if(on < 2000)
  {
    analogWrite(PIN_BUZZER, 255/2); //buzzer on
    delay(on); //wait ms
    analogWrite(PIN_BUZZER, 0); //buzzer off
  }

  return;
}


unsigned int light_sensor(void) //read the light sensor
{
  unsigned int i;
  uint32_t color = ws2812.getPixelColor(0); //save current color

  //ws2812.setPixelColor(2, COLOR_OFF); //LED 3 off
  ws2812.fill(COLOR_OFF, 0, 4); //all 4 LEDs off
  ws2812.show();

  digitalWrite(PIN_LSENSOR_PWR, HIGH); //light sensor on
  delay(40); //wait 40ms
  i = analogRead(PIN_LSENSOR); //0...1024
  delay(10); //wait 10ms
  i += analogRead(PIN_LSENSOR); //0...1024
  i /= 2;
  digitalWrite(PIN_LSENSOR_PWR, LOW); //light sensor off

  //ws2812.setPixelColor(2, color); //LED 3 on
  leds(color);

  return i;
}


float co2_sensor(void)
{
  return co2_value;
}


float temp_sensor(void)
{
  return temp_value;
}


float humi_sensor(void)
{
  return humi_value;
}


float pres_sensor(void)
{
  return pres_value;
}


unsigned int check_sensors(void) //read sensors
{
  if(features & FEATURE_SCD30)
  {
    if(scd30.dataAvailable())
    {
      co2_value  = scd30.getCO2();
      temp_value = scd30.getTemperature();
      humi_value = scd30.getHumidity();
      if(features & FEATURE_LPS22HB)
      {
        pres_value  = lps22.readPressure()*10; //kPa -> hPa
        temp2_value = lps22.readTemperature()-temp_offset;
      }
      if(features & FEATURE_BMP280)
      {
        pres_value  = bmp280.readPressure()/100; //Pa -> hPa
        temp2_value = bmp280.readTemperature()-temp_offset;
      }
      if((pres_value < (pres_last-PRESSURE_DIFF)) || (pres_value > (pres_last+PRESSURE_DIFF)))
      {
        pres_last = pres_value;
        scd30.setAmbientPressure(pres_value); //hPa=mBar
      }
      if(humi_value < 0)
      {
        humi_value = 0;
      }
      else if(humi_value > 100)
      {
        humi_value = 100;
      }
      return 1;
    }
  }
  else if(features & FEATURE_SCD4X)
  {
    uint16_t v_co2;
    float v_temp;
    float v_humi;
    if(scd4x.readMeasurement(v_co2, v_temp, v_humi) == 0)
    {
      co2_value  = v_co2;
      temp_value = v_temp;
      humi_value = v_humi;
      if(features & FEATURE_LPS22HB)
      {
        pres_value  = lps22.readPressure()*10; //kPa -> hPa
        temp2_value = lps22.readTemperature()-temp_offset;
      }
      if(features & FEATURE_BMP280)
      {
        pres_value  = bmp280.readPressure()/100; //Pa -> hPa
        temp2_value = bmp280.readTemperature()-temp_offset;
      }
      if((pres_value < (pres_last-PRESSURE_DIFF)) || (pres_value > (pres_last+PRESSURE_DIFF)))
      {
        pres_last = pres_value;
        scd4x.stopPeriodicMeasurement();
        delay(1000);
        scd4x.setAmbientPressure(pres_value); //hPa=mBar
        delay(500);
        scd4x.startPeriodicMeasurement();
      }
      if(humi_value < 0)
      {
        humi_value = 0;
      }
      else if(humi_value > 100)
      {
        humi_value = 100;
      }
      return 1;
    }
  }

  return 0;
}


void show_data(void) //display data
{
  if(features & FEATURE_USB)
  {
    Serial.print("c: ");           //CO2
    Serial.println(co2_value);     //value in ppm
    Serial.print("t: ");           //temperature
    Serial.println(temp_value, 1); //value in deg C
    Serial.print("h: ");           //humidity
    Serial.println(humi_value, 1); //value in %
    Serial.print("l: ");           //light
    Serial.println(light_value);
    if(features & (FEATURE_LPS22HB|FEATURE_BMP280))
    {
      Serial.print("p: ");         //pressure
      Serial.println(pres_value);  //value in hPa
      Serial.print("u: ");         //temperature
      Serial.println(temp2_value); //value in deg C
    }
    Serial.println();
  }

  if(features & FEATURE_SSD1306)
  {
    display.clearDisplay();
    display.setTextSize(5);
    display.setCursor(5,5);
    display.println(co2_value);
    display.setTextSize(1);
    display.setCursor(5,56);
    display.println("CO2 Level in ppm");
    display.display();
  }

  return;
}


void serial_service(void)
{
  static int calibration_done=0;
  int i, cmd, val;
  char tmp[32];

  if((features & FEATURE_USB) == 0)
  {
    return;
  }

  if(Serial.available() == 0)
  {
    return;
  }

  cmd = Serial.read(); //command
  if((cmd != 'R') && (remote_on == 0))
  {
    return;
  }

  val = Serial.read(); //write/read
  if(val == '=') //=
  {
    switch(toupper(cmd))
    {
      case 'R': //remote control
        cmd = Serial.read();
        if(cmd == '1') //on
        {
          remote_on = 1;
          buzzer(0); //buzzer off
          ws2812.setBrightness(30); //0...255
          leds(COLOR_VIOLET); //LEDs violet
          Serial.println("OK");
        }
        else if(cmd == '0') //off
        {
          remote_on = 0;
          calibration_done = 0;
          ws2812.setBrightness(settings.brightness);
          Serial.println("OK");
        }
        else if(cmd == 'R' && remote_on) //reset
        {
          Serial.println("OK");
          leds(0); //LEDs off
          Serial.flush();
          Serial.end();
          delay(20); //wait 20ms
          NVIC_SystemReset();
          while(1);
        }
        break;

      case 'S': //save
        cmd = Serial.read();
        if(cmd == '1')
        {
          settings.valid = true;
          flash_settings.write(settings); //save settings
          Serial.println("OK");
        }
        break;

      case 'H': //LED brightness
        i = Serial.readBytesUntil('\n', tmp, sizeof(tmp));
        if(i >= 0)
        {
          tmp[i] = 0;
          sscanf(tmp, "%X", &val);
          if(val < 0)
          {
            val = 0;
          }
          else if(val > 255)
          {
            val = 255;
          }
          settings.brightness = val;
          ws2812.setBrightness(val);
          ws2812.show();
          Serial.println("OK");
        }
        break;

      case 'L': //LED color
        i = Serial.readBytesUntil('\n', tmp, sizeof(tmp));
        if(i > 0)
        {
          tmp[i] = 0;
          sscanf(tmp, "%X", &val);
          leds(val);
          Serial.println("OK");
        }
        break;

      case 'B': //buzzer
        cmd = Serial.read();
        if(cmd == '1')
        {
          buzzer(500); //500ms buzzer on
          settings.buzzer = 1;
          Serial.println("OK");
        }
        else if(cmd == '0')
        {
          settings.buzzer = 0;
          Serial.println("OK");
        }
        break;

      case 'T': //temperature offset
        i = Serial.readBytesUntil('\n', tmp, sizeof(tmp));
        if(i > 0)
        {
          tmp[i] = 0;
          sscanf(tmp, "%d", &val);
          if((val >= 0) && (val <= 20))
          {
            temp_offset = val;
            if(features & FEATURE_SCD30)
            {
              scd30.setTemperatureOffset(val); //temperature offset
              Serial.println("OK");
            }
            else if(features & FEATURE_SCD4X)
            {
              scd4x.stopPeriodicMeasurement();
              delay(1000);
              if(scd4x.setTemperatureOffset(val) == 0) //temperature offset
              {
                Serial.println("OK");
              }
              else
              {
                Serial.println("ERROR");
              }
              delay(500);
              scd4x.startPeriodicMeasurement();
            }
          }
        }
        break;

      case 'A': //altitude above sea level
        i = Serial.readBytesUntil('\n', tmp, sizeof(tmp));
        if(i > 0)
        {
          tmp[i] = 0;
          sscanf(tmp, "%d", &val);
          if((val >= 0) && (val <= 3000))
          {
            if(features & FEATURE_SCD30)
            {
              scd30.setAltitudeCompensation(val); //meters above sea level
              Serial.println("OK");
            }
            else if(features & FEATURE_SCD4X)
            {
              scd4x.stopPeriodicMeasurement();
              delay(1000);
              if(scd4x.setSensorAltitude(val) == 0) //meters above sea level
              {
                Serial.println("OK");
              }
              else
              {
                Serial.println("ERROR");
              }
              delay(500);
              scd4x.startPeriodicMeasurement();
            }
          }
        }
        break;

      case 'C': //calibration
        i = Serial.readBytesUntil('\n', tmp, sizeof(tmp));
        if((i > 0) && (calibration_done == 0))
        {
          tmp[i] = 0;
          sscanf(tmp, "%d", &val);
          if((val > 0) && (val < 400))
          {
            val = 400;
          }
          if((val >= 400) || (val <= 2000))
          {
            calibration_done = 1;
            if(features & FEATURE_SCD30)
            {
              scd30.setForcedRecalibrationFactor(val);
              delay(500);
            }
            else if(features & FEATURE_SCD4X)
            {
              uint16_t corr;
              scd4x.stopPeriodicMeasurement();
              delay(1000);
              scd4x.performForcedRecalibration(val, corr);
              delay(1000);
              scd4x.startPeriodicMeasurement();
            }
            Serial.println("OK");
          }
        }
        break;

      case '1': //range 1
      case '2': //range 2
      case '3': //range 3
      case '4': //range 4
      case '5': //range 5
        i = Serial.readBytesUntil('\n', tmp, sizeof(tmp));
        if(i > 0)
        {
          tmp[i] = 0;
          sscanf(tmp, "%d", &val);
          if((val >= 400) && (val <= 10000))
          {
            settings.range[cmd-'1'] = val;
            Serial.println("OK");
          }
        }
        break;
    }
  }
  else if(val == '?') //?
  {
    switch(toupper(cmd))
    {
      case 'V': //version
        Serial.println(VERSION);
        break;
      case 'H': //LED brightness
        Serial.println(settings.brightness, HEX);
        break;
      case 'B': //buzzer
        Serial.println(settings.buzzer, DEC);
        break;
      case 'T': //temperature offset
        if(features & FEATURE_SCD30)
        {
          val = scd30.getTemperatureOffset();
        }
        else if(features & FEATURE_SCD4X)
        {
          float offset;
          scd4x.stopPeriodicMeasurement();
          delay(500);
          scd4x.getTemperatureOffset(offset);
          delay(500);
          scd4x.startPeriodicMeasurement();
          val = offset;
        }
        Serial.println(val, DEC);
        break;
      case 'A': //altitude above sea level
        if(features & FEATURE_SCD30)
        {
          val = scd30.getAltitudeCompensation();
        }
        else if(features & FEATURE_SCD4X)
        {
          uint16_t alt;
          scd4x.stopPeriodicMeasurement();
          delay(500);
          scd4x.getSensorAltitude(alt);
          delay(500);
          scd4x.startPeriodicMeasurement();
          val = alt;
        }
        Serial.println(val, DEC);
        break;
      case '1': //range 1
      case '2': //range 2
      case '3': //range 3
      case '4': //range 4
      case '5': //range 5
        Serial.println(settings.range[cmd-'1'], DEC);
        break;
    }
  }

  return;
}


void urldecode(char *src) //decode URL parameters
{
  char a, b, *dst = src;

  while(*src)
  {
    if((*src == '%') && ((a = src[1]) && (b = src[2])) && (isxdigit((uint8_t)a) && isxdigit((uint8_t)b)))
    {
      if (a >= 'a')
        a -= 'a' - 'A';

      if (a >= 'A')
        a -= 'A' - 10;
      else
        a -= '0';

      if (b >= 'a')
        b -= 'a' - 'A';

      if (b >= 'A')
        b -= 'A' - 10;
      else
        b -= '0';

      *dst++ = 16 * a + b;
      src += 3;
    }
    else if (*src == '+')
    {
      *dst++ = ' ';
      src++;
    }
    else
    {
      *dst++ = *src++;
    }
  }
  *dst++ = '\0';

  return;
}


void webserver_service(void)
{
  static unsigned long t_check=0;
  unsigned int status;

  if((features & FEATURE_WINC1500) == 0)
  {
    return;
  }

  status = WiFi.status();

  if(status == WL_IDLE_STATUS) //connecting
  {
    return;
  }
  else if((status == WL_CONNECT_FAILED) ||
          (status == WL_CONNECTION_LOST) ||
          (status == WL_DISCONNECTED)) //connection lost
  {
    if((millis()-t_check) > (1*60000UL)) //1min
    {
      t_check = millis();
      wifi_start();
    }
    return;
  }

  t_check = millis(); //save time for reconnect after 1min

  WiFiClient client = server.available();
  if(!client) //client not connected
  {
    return;
  }
  //if(features & FEATURE_USB)
  //{
  //  Serial.println("WiFi client connected");
  //}
  boolean currentLineIsBlank=true;
  unsigned int pos=0;
  char buf[1024];
  char req[2][64+1]; //HTTP request
  req[0][0] = 0;
  while(client.connected())
  {
    if((millis()-t_check) > (5*1000UL)) //stop after 5s
    {
      break;
    }
    if(client.available())
    {
      char c = client.read();
      if(c == '\n' && currentLineIsBlank) //end of header
      {
        if(strncmp(req[0], "GET ", 4) && strncmp(req[0], "POST ", 5)) //no GET or POST
        {
          sprintf(buf,
              "HTTP/1.1 400 Bad Request\r\n" \
              "Content-Type: text/plain\r\n" \
              "Connection: close\r\n" \
              "\r\n" \
              "400 Bad Request\r\n"
          );
          client.print(buf);
        }
        else if(strncmp(req[0], "GET /json", 9) == 0) //JSON
        {
          if(features & (FEATURE_LPS22HB|FEATURE_BMP280))
          {
            sprintf(buf,
                "HTTP/1.1 200 OK\r\n" \
                "Content-Type: application/json\r\n" \
                "Connection: close\r\n" \
                "\r\n" \
                "{\r\n" \
                " \"c\": %i,\r\n" \
                " \"t\": %.1f,\r\n" \
                " \"h\": %.1f,\r\n" \
                " \"p\": %.1f,\r\n" \
                " \"u\": %.1f,\r\n" \
                " \"l\": %i\r\n" \
                "}\r\n",
                co2_value, temp_value, humi_value, pres_value, temp2_value, light_value
            );
          }
          else
          {
            sprintf(buf,
                "HTTP/1.1 200 OK\r\n" \
                "Content-Type: application/json\r\n" \
                "Connection: close\r\n" \
                "\r\n" \
                "{\r\n" \
                " \"c\": %i,\r\n" \
                " \"t\": %.1f,\r\n" \
                " \"h\": %.1f,\r\n" \
                " \"l\": %i\r\n" \
                "}\r\n",
                co2_value, temp_value, humi_value, light_value
            );
          }
          client.print(buf);
        }
        else if(strncmp(req[0], "GET /cmk-agent", 14) == 0) //Checkmk agent
        {
          //CO2-Ampels can be added directly to checkmk.com monitoring this way.
          //Plugins are not strictly required.
          //Since HTTP is used as the transport, use "Data Source":
          //wget -O - http://ip_address/cmk-agent
          //See: https://docs.checkmk.com/latest/en/datasource_programs.html
          if(features & (FEATURE_LPS22HB|FEATURE_BMP280))
          {
            sprintf(buf,
                "HTTP/1.1 200 OK\r\n" \
                "Content-Type: text/plain\r\n" \
                "Connection: close\r\n" \
                "\r\n" \
                //Plaintext in the format expected by Checkmk
                //See: https://docs.checkmk.com/latest/en/devel_check_plugins.html
                "<<<check_mk>>>\r\n" \
                "AgentOS: arduino\r\n" \
                //Check plugin required on the server to evaluate the metrics
                "<<<watterott_co2ampel_plugin>>>\r\n" \
                "co2 %i\r\n" \
                "temp %.1f\r\n" \
                "humidity %.1f\r\n" \
                "lighting %i\r\n" \
                "pressure %.1f\r\n" \
                "temp2 %.1f\r\n" \
                //Ad-hoc check that needs no server plugin, uses the ampel thresholds.
                //Note: only one line - the Checkmk server performs the evaluation
                //itself using the supplied thresholds. We read them from the ampel here:
                "<<<local:sep(0)>>>\r\n" \
                "P \"CO2 level (ppm)\" co2ppm=%i;%i;%i CO2/ventilation control with Watterott CO2-Ampel, thresholds taken from sensor board.\r\n",
                co2_value, temp_value, humi_value, light_value, pres_value, temp2_value,
                co2_value, settings.range[1], settings.range[2]
            );
          }
          else
          {
            sprintf(buf,
                "HTTP/1.1 200 OK\r\n" \
                "Content-Type: text/plain\r\n" \
                "Connection: close\r\n" \
                "\r\n" \
                //Plaintext in the format expected by Checkmk
                //See: https://docs.checkmk.com/latest/en/devel_check_plugins.html
                "<<<check_mk>>>\r\n" \
                "AgentOS: arduino\r\n" \
                //Check plugin required on the server to evaluate the metrics
                "<<<watterott_co2ampel_plugin>>>\r\n" \
                "co2 %i\r\n" \
                "temp %.1f\r\n" \
                "humidity %.1f\r\n" \
                "lighting %i\r\n" \
                //Ad-hoc check that needs no server plugin, uses the ampel thresholds.
                //Note: only one line - the Checkmk server performs the evaluation
                //itself using the supplied thresholds. We read them from the ampel here:
                "<<<local:sep(0)>>>\r\n" \
                "P \"CO2 level (ppm)\" co2ppm=%i;%i;%i CO2/ventilation control with Watterott CO2-Ampel, thresholds taken from sensor board.\r\n",
                co2_value, temp_value, humi_value, light_value,
                co2_value, settings.range[1], settings.range[2]
            );
          }
          client.print(buf);
        }
        else if(strncmp(req[0], "GET /favicon", 12) == 0) //favicon
        {
          sprintf(buf,
              "HTTP/1.1 404 Not Found\r\n" \
              "Content-Type: text/plain\r\n" \
              "Connection: close\r\n" \
              "\r\n" \
              "404 Not Found\r\n"
          );
          client.print(buf);
        }
        else
        {
          //process HTTP POST data
          if((strncmp(req[0], "POST ", 5) == 0) && client.available())
          {
            req[0][0] = 0; //SSID
            req[1][0] = 0; //code
            for(unsigned int r=0, i=0, last_c=0; client.available();)
            {
              c = client.read();
              if(c == '&') //format: 1=xxx&2=yyy
              {
                r = 0;
              }
              else if((c == '=') && isdigit(last_c)) //1=xxx
              {
                r = last_c-'0';
                i = 0;
              }
              else if((r > 0) && (r < 3)) //1 to 2
              {
                req[r-1][i++] = c;
                req[r-1][i] = 0;
              }
              last_c = c;
            }
            urldecode(req[0]); //Serial.println(req[0]);
            urldecode(req[1]); //Serial.println(req[1]);
            if(strcmp(req[0], settings.wifi_ssid) || strcmp(req[1], settings.wifi_code))
            {
              //todo: strip trailing whitespace
              strcpy(settings.wifi_ssid, req[0]);
              strcpy(settings.wifi_code, req[1]);
              flash_settings.write(settings); //save settings
            }
          }
          //HTTP header+data
          sprintf(buf,
              "HTTP/1.1 200 OK\r\n" \
              "Content-Type: text/html\r\n" \
              "Connection: close\r\n" \
              "\r\n"
              "<!DOCTYPE html>\r\n" \
              "<html>\r\n" \
              "<head>\r\n" \
              "<meta charset=utf-8>\r\n" \
              "<meta http-equiv=refresh content=120>\r\n" \
              "<title>CO2-Ampel</title>\r\n" \
              "<link rel=icon href=\"data:image/gif;base64,R0lGODlhAQABAAAAACwAAAAAAQABAAA=\">\r\n" \
              "<style>\r\n" \
              "body { font-size:1.0em; font-family:Lato,sans-serif; padding:10px; }\r\n" \
              "#data { font-size:3.0em; }\r\n" \
              "#wifi { font-size:1.0em; display:none; }\r\n" \
              "#info { font-size:0.9em; }\r\n" \
              "</style>\r\n" \
              "<script>\r\n" \
              "function wifi() {\r\n" \
              "var box = document.getElementById('wifi');\r\n" \
              "if(box.style.display != 'block') { box.style.display = 'block'; }\r\n" \
              "else { box.style.display = 'none'; }\r\n" \
              "}\r\n" \
              "</script>\r\n" \
              "</head>\r\n" \
              "<body>\r\n"
          );
          client.print(buf);

          if(features & (FEATURE_LPS22HB|FEATURE_BMP280))
          {
            sprintf(buf,
                "<div id=data>\r\n" \
                "CO2 (ppm): %i<br/>\r\n" \
                "Temperature (&deg;C): %.1f<br/>\r\n" \
                "Humidity (% rel): %.1f<br/>\r\n" \
                "Pressure (hPa): %.1f<br/>\r\n" \
                "Temperature (&deg;C): %.1f<br/>\r\n" \
                "</div>\r\n",
                co2_value, temp_value, humi_value, pres_value, temp2_value
            );
          }
          else
          {
            sprintf(buf,
                "<div id=data>\r\n" \
                "CO2 (ppm): %i<br/>\r\n" \
                "Temperature (&deg;C): %.1f<br/>\r\n" \
                "Humidity (% rel): %.1f<br/>\r\n" \
                "</div>\r\n",
                co2_value, temp_value, humi_value
            );
          }
          client.print(buf);

          String fv = WiFi.firmwareVersion();
          byte mac[6];
          WiFi.macAddress(mac);
          sprintf(buf,
              "<br/><br/>\r\n" \
              "<a href='/json'>JSON</a> - <a href='/cmk-agent'>Checkmk</a> - <a href='#' onclick='wifi();'>WiFi Login</a>\r\n" \
              "<br/><br/>\r\n" \
              "<div id=wifi>\r\n" \
              "<form method=post>\r\n" \
              "SSID <input name=1 size=30 maxlength=64 placeholder=SSID value='%s'><br/>\r\n" \
              "Code <input name=2 size=30 maxlength=64 placeholder=Password value=''><br/>\r\n" \
              "<input type=submit> (requires reboot)<br/>\r\n" \
              "</form><br/>\r\n" \
              "<div id=info>\r\n" \
              "Firmware: v" VERSION ", \r\n" \
              "WINC1500: %s, \r\n" \
              "MAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n" \
              "</div>\r\n" \
              "</div>\r\n" \
              "</body>\r\n" \
              "</html>\r\n",
              settings.wifi_ssid, /*settings.wifi_code, */ fv.c_str(),
              mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]
          );
          client.print(buf);
        }
        break;
      }
      else //save request
      {
        if(pos < (sizeof(req[0])-1))
        {
          req[0][pos++] = c;
          req[0][pos] = 0;
        }
      }
      if(c == '\n')
      {
        currentLineIsBlank = true;
      }
      else if(c != '\r')
      {
        currentLineIsBlank = false;
      }
    }
  }

  delay(20); //wait 20ms to send
  client.stop();

  return;
}


int check_i2c(Sercom *sercom, byte addr) //1=okay
{
  int res = 0;

  for(int t=3; (t!=0) && (res==0); t--) //try 3 times
  {
    sercom->I2CM.CTRLA.bit.ENABLE = 1; //enable master mode
    delay(10); //wait 10ms
    sercom->I2CM.ADDR.bit.ADDR = (addr<<1) | 0x00; //start transfer
    delay(10); //wait 10ms
    if(sercom->I2CM.INTFLAG.bit.MB || sercom->I2CM.INTFLAG.bit.SB) //data transmitted
    {
      if(!sercom->I2CM.STATUS.bit.RXNACK) //ack received
      {
        res = 1; //ok
        break;
      }
    }
  }

  /*
  if(res == 0)
  {
    if(sercom == SERCOM1)
    {
      Wire.beginTransmission(addr);
      if(Wire.endTransmission() == 0)
      {
        res = 1; //ok
      }
    }
    else if(sercom == SERCOM2)
    {
      Wire1.beginTransmission(addr);
      if(Wire1.endTransmission() == 0)
      {
        res = 1; //ok
      }
    }
  }
  */

  return res;
}


void self_test(void) //test program
{
  //buzzer test
  buzzer(500); //500ms buzzer on

  //LED test
  leds(0xFF0000); //LEDs red
  delay(1000); //wait 1s
  leds(0x00FF00); //LEDs green
  delay(1000); //wait 1s
  leds(0x0000FF); //LEDs blue
  delay(1000); //wait 1s
  leds(COLOR_OFF); //LEDs off

  #if WIFI_AMPEL
    //ATWINC1500 test
    if(WiFi.status() == WL_NO_SHIELD) //ATWINC1500 error
    {
      if(features & FEATURE_USB)
      {
        Serial.println("Error: ATWINC1500");
      }
      while(1)
      {
        leds(COLOR_RED); //LEDs red
        delay(500); //wait 500ms
        leds(COLOR_YELLOW); //LEDs yellow
        delay(500); //wait 500ms
      }
    }
    else
    {
      leds(COLOR_WHITE); //LEDs white
      buzzer(1000); //1s buzzer on
    }
  #endif

  //RFM9X test
  #if PRO_AMPEL
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    digitalWrite(20, LOW); //RFM9X CS low/active
    SPI.transfer(0x42); //0x42 = version
    byte i = SPI.transfer(0x00);
    digitalWrite(20, HIGH); //RFM9X CS high
    if(i == 0x12) //check version
    {
      leds(COLOR_WHITE); //LEDs white
      buzzer(1000); //1s buzzer on
    }
  #endif

  //sensor test
  unsigned int co2, light;
  float temp, humi, pres;
  co2_value  = 0;
  temp_value = 0;
  humi_value = 0;
  #if PRO_AMPEL
    pres_value = 0;
  #endif
  ws2812.fill(COLOR_OFF, 0, 4); //LEDs off
  for(unsigned int okay=0; okay < 15;)
  {
    if(digitalRead(PIN_SWITCH) == 0) //button pressed?
    {
      break; //abort
    }
    status_led(200); //status LED

    digitalWrite(PIN_LSENSOR_PWR, HIGH); //light sensor on
    delay(50); //wait 50ms
    light = analogRead(PIN_LSENSOR); //0...1024
    digitalWrite(PIN_LSENSOR_PWR, LOW); //light sensor off
    if((light >= 50) && (light <= 1000)) //50-1000
    {
      okay |= (1<<0);
      ws2812.setPixelColor(0, COLOR_GREEN);
    }
    else
    {
      okay &= ~(1<<0);
      ws2812.setPixelColor(0, COLOR_OFF);
    }

    if(check_sensors())
    {
      co2  = co2_sensor();
      temp = temp_sensor();
      humi = humi_sensor();
      pres = pres_sensor();

      if((co2 >= 100) && (co2 <= 1500)) //100-1500ppm
      {
        okay |= (1<<1);
        ws2812.setPixelColor(1, COLOR_GREEN);
      }
      else
      {
        okay &= ~(1<<1);
        ws2812.setPixelColor(1, COLOR_OFF);
      }

      if(((temp >=   5) && (temp <=   35)) && //5-35 deg C
         ((pres >= 700) && (pres <= 1400)))   //700-1400 hPa
      {
        okay |= (1<<2);
        ws2812.setPixelColor(2, COLOR_GREEN);
      }
      else
      {
        okay &= ~(1<<2);
        ws2812.setPixelColor(2, COLOR_OFF);
      }

      if((humi >= 20) && (humi <= 80)) //20-80%
      {
        okay |= (1<<3);
        ws2812.setPixelColor(3, COLOR_GREEN);
      }
      else
      {
        okay &= ~(1<<3);
        ws2812.setPixelColor(3, COLOR_OFF);
      }

      show_data();
    }

    ws2812.show();
  }

  delay(2000); //wait 2s

  return;
}


void air_test(void) //fresh-air test
{
  unsigned int co2;

  ws2812.fill(COLOR_WHITE, 0, 4); //LEDs white
  ws2812.show();

  while(1)
  {
    if(digitalRead(PIN_SWITCH) == 0) //button pressed?
    {
      break; //abort
    }

    status_led(200); //status LED

    if(check_sensors())
    {
      co2 = co2_sensor();

      if(co2 < 300)
      {
        ws2812.fill(COLOR_RED, 0, NUM_LEDS); //red
      }
      else if(co2 < 350)
      {
        ws2812.fill(COLOR_YELLOW, 0, NUM_LEDS); //yellow
      }
      else if(co2 <= 450)
      {
        ws2812.fill(COLOR_BLUE, 0, NUM_LEDS); //blue
      }
      else if(co2 <= 500)
      {
        ws2812.fill(COLOR_YELLOW, 0, NUM_LEDS); //yellow
      }
      else //>500
      {
        ws2812.fill(COLOR_RED, 0, NUM_LEDS); //red
      }
      ws2812.show();

      show_data();
    }
  }

  //end
  leds(COLOR_OFF);//LEDs off
  buzzer(250); //250ms buzzer on

  return;
}


unsigned int select_value(unsigned int value, unsigned int min, unsigned int max, unsigned int fill, uint32_t color, uint32_t color_off)
{
  unsigned int timeout, sw;

  ws2812.fill(color_off, 0, 4);
  if(fill == 0)
  {
    ws2812.setPixelColor(value, COLOR_VIOLET);
  }
  else if(value > 0)
  {
    ws2812.fill(color, 0, value);
  }
  ws2812.show();

  for(sw=0, timeout=0; timeout<1000; timeout++) //10s timeout
  {
    delay(10); //wait 10ms

    if(digitalRead(PIN_SWITCH) == LOW) //button pressed
    {
      status_led(1); //status LED on
      sw++;
      if(sw > 200)
      {
        leds(COLOR_OFF); //LEDs off
      }
      timeout = 0;
    }
    else //button released
    {
      status_led(0); //status LED off
      if(sw > 200) //2s button press
      {
        break;
      }
      else if(sw > 10) //100ms button press
      {
        value++;
        if(value > max)
        {
          value = min;
        }
        ws2812.fill(color_off, 0, 4);
        if(fill == 0)
        {
          ws2812.setPixelColor(value, color);
        }
        else if(value > 0)
        {
          ws2812.fill(color, 0, value);
        }
        ws2812.show();
      }
      sw = 0;
    }
  }

  leds(COLOR_OFF); //LEDs off
  delay(500); //wait 500ms

  return value;
}


void altitude_toffset(void) //altitude and temperature offset
{
  unsigned int timeout, sw, value=0;

  //altitude
  if(features & FEATURE_SCD30)
  {
    value = scd30.getAltitudeCompensation() / 250; //meters above sea level
  }
  else if(features & FEATURE_SCD4X)
  {
    uint16_t altitude;
    scd4x.stopPeriodicMeasurement();
    delay(500);
    scd4x.getSensorAltitude(altitude); //meters above sea level
    value = altitude/250;
  }

  value = select_value(value, 0, 4, 1, COLOR_RED, COLOR_WHITE) * 250;

  if(features & FEATURE_SCD30)
  {
    scd30.setAltitudeCompensation(value); //meters above sea level
  }
  else if(features & FEATURE_SCD4X)
  {
    scd4x.setSensorAltitude(value); //meters above sea level
  }

  if(features & FEATURE_USB)
  {
    Serial.print("Altitude: ");
    Serial.println(value, DEC);
  }

  //temperature offset
  if(features & FEATURE_SCD30)
  {
    value = scd30.getTemperatureOffset() / 2; //temperature offset
  }
  else if(features & FEATURE_SCD4X)
  {
    float offset;
    scd4x.getTemperatureOffset(offset); //meters above sea level
    value = offset / 2;
  }

  value = select_value(value, 0, 4, 1, COLOR_YELLOW, COLOR_BLUE) * 2;

  if(features & FEATURE_SCD30)
  {
    scd30.setTemperatureOffset(value); //temperature offset
  }
  else if(features & FEATURE_SCD4X)
  {
    scd4x.setTemperatureOffset(value); //temperature offset
    delay(500);
    scd4x.startPeriodicMeasurement();
  }

  if(features & FEATURE_USB)
  {
    Serial.print("Temperature: ");
    Serial.println(value, DEC);
  }

  //buzzer
  settings.buzzer = select_value(settings.buzzer, 0, 1, 1, COLOR_GREEN, COLOR_WHITE);

  if(features & FEATURE_USB)
  {
    Serial.print("Buzzer: ");
    Serial.println(settings.buzzer, DEC);
  }

  //end
  flash_settings.write(settings); //save settings
  leds(COLOR_BLUE);//LEDs blue
  buzzer(250); //250ms buzzer on

  return;
}


void calibration(void) //calibration
{
  unsigned int abort=0, cycle, again, interval=INTERVAL, co2, co2_last;

  //The measurement interval during calibration and in operation should be the same.
  //Different intervals can lead to deviations and fluctuating measured values.
  //scd30.setMeasurementInterval(INTERVAL); //set measurement interval

  ws2812.fill(COLOR_WHITE, 0, 4); //LEDs white
  ws2812.show();

  if(features & FEATURE_SCD4X)
  {
    interval = 5; //5s
  }

  //ASC
  if(features & FEATURE_SCD30)
  {
    if(AUTO_CALIBRATION) //ASC on
    {
      if(scd30.getAutoSelfCalibration() == 0)
      {
        scd30.setAutoSelfCalibration(1);
      }
    }
    else //ASC off
    {
      if(scd30.getAutoSelfCalibration() != 0)
      {
        scd30.setAutoSelfCalibration(0);
      }
    }
  }
  else if(features & FEATURE_SCD4X)
  {
    scd4x.stopPeriodicMeasurement();
    delay(1000);
    if(AUTO_CALIBRATION) //ASC on
    {
      uint16_t asc;
      scd4x.getAutomaticSelfCalibration(asc);
      if(asc == 0)
      {
        scd4x.setAutomaticSelfCalibration(1);
      }
    }
    else //ASC off
    {
      uint16_t asc;
      scd4x.getAutomaticSelfCalibration(asc);
      if(asc != 0)
      {
        scd4x.setAutomaticSelfCalibration(0);
      }
    }
    delay(500);
    scd4x.startPeriodicMeasurement();
  }

  calibration_start:

  //calibration
  co2 = co2_last = co2_sensor();
  for(again=0, cycle=0; cycle < (180/interval);) //at least 3 minutes
  {
    if(digitalRead(PIN_SWITCH) == 0) //button pressed?
    {
      abort = 1;
      break; //abort
    }

    status_led(200); //status LED

    if(check_sensors())
    {
      co2 = co2_sensor();
      if((co2 >= 200) && (co2 <= 800) &&
         (co2 >= (co2_last-30)) &&
         (co2 <= (co2_last+30))) //+/-30ppm tolerance from the previous value
      {
        cycle++;
        again = 0;
      }
      else //sensor incorrectly calibrated
      {
        again++;
        if(again > 3)
        {
          again = 1;
          cycle++;
        }
      }
      co2_last = co2;

      if(co2 <= 500)
      {
        ws2812.fill(COLOR_BLUE, 2, 2); //blue
      }
      else if(co2 <= 750)
      {
        ws2812.fill(COLOR_GREEN, 2, 2); //green
      }
      else if(co2 <= 1500)
      {
        ws2812.fill(COLOR_YELLOW, 2, 2); //yellow
      }
      else //>1500
      {
        ws2812.fill(COLOR_RED, 2, 2); //red
      }
      ws2812.show();

      if(features & FEATURE_USB)
      {
        Serial.print("loop: ");
        Serial.println(cycle);
      }

      show_data();
    }
  }
  if(abort == 0)
  {
    if(features & FEATURE_SCD30)
    {
      scd30.setForcedRecalibrationFactor(400); //400ppm = fresh air
      delay(500);
    }
    else if(features & FEATURE_SCD4X)
    {
      uint16_t corr;
      scd4x.stopPeriodicMeasurement();
      delay(1000);
      scd4x.performForcedRecalibration(400, corr); //400ppm = fresh air
      delay(1000);
      scd4x.startPeriodicMeasurement();
    }
    if(again != 0)
    {
      Serial.println("Restart calibration");
      goto calibration_start;
    }
    leds(COLOR_BLUE);//LEDs blue
    buzzer(500); //500ms buzzer on
    if(features & FEATURE_USB)
    {
      Serial.println("Calibration OK");
    }
    delay(3000); //wait 3s
  }

  return;
}


void menu(void)
{
  unsigned int timeout, sw, value;

  ws2812.setBrightness(30); //0...255
  leds(COLOR_VIOLET); //LEDs violet
  delay(500); //wait 500ms
  leds(COLOR_OFF); //LEDs off
  buzzer(250); //250ms buzzer on

  value = select_value(1, 1, 4, 1, COLOR_VIOLET, ws2812.Color(20,20,20));

  switch(value)
  {
    case 1: self_test();        break;
    case 2: air_test();         break;
    case 3: altitude_toffset(); break;
    case 4: calibration();      break;
  }

  ws2812.setBrightness(settings.brightness); //0...255
  leds(ws2812.Color(20,20,20));//LEDs white

  return;
}


unsigned int wifi_start_ap(void)
{
  byte mac[6];
  char ssid[32];

  if(features & FEATURE_USB)
  {
    Serial.println("WiFi AP start...");
  }

  WiFi.macAddress(mac); //query MAC address
  sprintf(ssid, "CO2AMPEL-%X-%X", mac[1], mac[0]);

  if(WiFi.status() != WL_IDLE_STATUS)
  {
    WiFi.end(); //WiFi.disconnect();
    //reset_mcu();
  }

  WiFi.hostname(ssid); //set hostname
  if(WiFi.beginAP(ssid) != WL_AP_LISTENING)
  {
    WiFi.end();
    return 1;
  }

  delay(5000); //wait 5s

  server.begin(); //start webserver

  return 0;
}


unsigned int wifi_start(void)
{
  byte mac[6];
  char name[32];

  if(settings.wifi_ssid[0] == 0) //no login data
  {
    return 1;
  }

  if(features & FEATURE_USB)
  {
    Serial.println("WiFi connect...");
  }

  WiFi.macAddress(mac); //query MAC address
  sprintf(name, "CO2AMPEL-%X-%X", mac[1], mac[0]);

  if(WiFi.status() != WL_IDLE_STATUS)
  {
    WiFi.end(); //WiFi.disconnect();
    //reset_mcu();
  }

  WiFi.hostname(name); //set hostname
  if(settings.ip_local[0] != 0) //static IP
  {
    WiFi.config(settings.ip_local, settings.ip_dns, settings.ip_gw, settings.netmask);  //set IP
  }
  if(strlen(settings.wifi_code) > 0) //password
  {
    WiFi.begin(settings.wifi_ssid, settings.wifi_code); //connect to WiFi network with password
  }
  else
  {
    WiFi.begin(settings.wifi_ssid); //connect to WiFi network without password
  }

  //wait for connection
  for(unsigned int t=0; WiFi.status() == WL_IDLE_STATUS; t++)
  {
    if(t >= 6) //6s
    {
      break;
    }
    status_led(1000); //status LED
  }

  if(!(WiFi.status() == WL_CONNECTED)) //connection failed
  {
    return 1;
  }

  server.begin(); //start webserver

  return 0;
}


void reset_mcu(void)
{
  if(features & FEATURE_USB)
  {
    Serial.println("Reset...");
  }

  status_led(0);
  buzzer(0);
  ws2812.setBrightness(BRIGHTNESS_DARK); //dark
  leds(COLOR_WHITE); //LEDs white

  if(features & FEATURE_WINC1500)
  {
    WiFi.end(); //WiFi.disconnect();
  }
  if(features & FEATURE_SCD30)
  {
    scd30.StopMeasurement();
    //scd30.reset; //soft reset
  }
  if(features & FEATURE_SCD4X)
  {
    scd4x.stopPeriodicMeasurement();
  }

  Wire.end();
  Wire1.end();
  Serial.end();

  __disable_irq(); //disable interrupts
  NVIC_SystemReset(); //reset
  while(1);
}


void setup()
{
  int run_menu=0;

  //set pins
  pinMode(6, INPUT_PULLUP); //PA08 SDA1
  pinMode(7, INPUT_PULLUP); //PA09 SCL1
  pinMode(8, INPUT_PULLUP); //PA12 SDA2
  pinMode(9, INPUT_PULLUP); //PA13 SCL2
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW); //LED off
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_WS2812, OUTPUT);
  digitalWrite(PIN_WS2812, LOW);
  pinMode(PIN_LSENSOR_PWR, OUTPUT);
  digitalWrite(PIN_LSENSOR_PWR, LOW); //light sensor off
  pinMode(PIN_LSENSOR, INPUT);
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  pinMode(14, OUTPUT); //PA18 WINC1500 CS pin
  digitalWrite(14, HIGH); //WINC1500 CS high
  pinMode(20, OUTPUT); //PA21 RFM9X CS pin
  digitalWrite(20, HIGH); //RFM9X CS high

  if(digitalRead(PIN_SWITCH) == LOW) //button pressed
  {
    run_menu = 1;
  }

  //WS2812
  ws2812.begin();
  ws2812.setBrightness(BRIGHTNESS); //0...255
  ws2812.fill(COLOR_OFF, 0, NUM_LEDS); //LEDs off
  ws2812.fill(ws2812.Color(20,20,20), 0, 4); //4 LEDs white
  ws2812.show();

  //Wire/I2C
  Wire.begin();
  Wire.setClock(50000); //50kHz, recommended for SCD30
  Wire1.begin();
  Wire1.setClock(100000); //100kHz ATECC+LPS22HB+BMP280

  //serial interface (USB)
  Serial.begin(BAUDRATE); //start serial port
  Serial.setTimeout(500); //500ms read timeout
  //while(!Serial); //wait for USB connection

  delay(250); //wait 250ms

  #if WIFI_AMPEL
    //ATWINC1500
    if(WiFi.status() != WL_NO_SHIELD) //ATWINC1500 found
    {
      features |= FEATURE_WINC1500;
    }
  #endif

  //LPS22HB
  if(check_i2c(SERCOM2, ADDR_LPS22HB)) //LPS22HB found
  {
    if(lps22.begin())
    {
      features |= FEATURE_LPS22HB;
    }
  }

  //BMP280
  if(check_i2c(SERCOM2, ADDR_BMP280)) //BMP280 found
  {
    if(bmp280.begin(ADDR_BMP280))
    {
      features |= FEATURE_BMP280;
    }
  }
  else if(check_i2c(SERCOM2, ADDR_BMP280+1)) //BMP280 found
  {
    if(bmp280.begin(ADDR_BMP280+1))
    {
      features |= FEATURE_BMP280;
    }
  }

  //SSD1306
  if(check_i2c(SERCOM0, ADDR_SSD1306)) //SSD1306 found
  {
    features |= FEATURE_SSD1306;
    delay(500); //wait 500ms
    display.begin(SSD1306_SWITCHCAPVCC, ADDR_SSD1306);
    display.clearDisplay();
    display.setTextColor(WHITE, BLACK);
    display.setTextSize(3);
    display.setCursor(40, 0);
    display.print("CO2");
    display.setCursor(23, 23);
    display.print("Ampel");
    display.setTextSize(1);
    display.setCursor(5, 48);
    display.print("Watterott electronic");
    display.setCursor(12, 56);
    display.print("www.watterott.com");
    display.display();
  }

  //SCD30+SCD4X
  if(check_i2c(SERCOM0, ADDR_SCD30)) //SCD30 found
  {
    for(int t=5; t!=0; t--) //try 5 times
    {
      Wire.begin();
      if(scd30.begin(Wire, AUTO_CALIBRATION))
      {
        features |= FEATURE_SCD30;
        break;
      }
      status_led(1000); //status LED
    }
    scd30.setMeasurementInterval(INTERVAL); //set measurement interval
    //scd30.setAmbientPressure(1000); //0 or 700-1400, air pressure in hPa
  }
  if(check_i2c(SERCOM0, ADDR_SCD4X)) //SCD4X found
  {
    for(int t=5; t!=0; t--) //try 5 times
    {
      Wire.begin();
      scd4x.begin(Wire);
      scd4x.stopPeriodicMeasurement();
      delay(100);
      if(scd4x.startPeriodicMeasurement() == 0)
      {
        features |= FEATURE_SCD4X;
        break;
      }
      status_led(1000); //status LED
    }
  }

  //temperature offset
  if(features & FEATURE_SCD30)
  {
    temp_offset = scd30.getTemperatureOffset();
  }
  else if(features & FEATURE_SCD4X)
  {
    float offset;
    scd4x.stopPeriodicMeasurement();
    delay(500);
    if(scd4x.getTemperatureOffset(offset) == 0)
    {
      temp_offset = offset;
    }
    delay(500);
    scd4x.startPeriodicMeasurement();
  }
  if(temp_offset >= 20)
  {
    temp_offset = TEMP_OFFSET;
  }

  //settings
  settings = flash_settings.read(); //read settings
  if((settings.valid == false) || (settings.brightness > 255) || (settings.range[0] < 100))
  {
    settings.brightness   = BRIGHTNESS;
    settings.range[0]     = START_GREEN;
    settings.range[1]     = START_YELLOW;
    settings.range[2]     = START_RED;
    settings.range[3]     = START_RED_BLINK;
    settings.range[4]     = START_BUZZER;
    settings.buzzer       = BUZZER;
    settings.wifi_ssid[0] = 0;
    strcpy(settings.wifi_ssid, WIFI_SSID);
    settings.wifi_code[0] = 0;
    strcpy(settings.wifi_code, WIFI_CODE);
    settings.netmask      = IPAddress(WIFI_NM);
    settings.ip_local     = IPAddress(WIFI_IP);
    settings.ip_gw        = IPAddress(WIFI_GW);
    settings.ip_dns       = IPAddress(WIFI_DNS);
    settings.valid        = true;
    flash_settings.write(settings);
    //default temperature offset
    if(features & FEATURE_WINC1500)
    {
      if(features & (FEATURE_LPS22HB|FEATURE_BMP280))
      {
        temp_offset = TEMP_OFFSET_PRO;
      }
      else
      {
        temp_offset = TEMP_OFFSET_WIFI;
      }
    }
    else
    {
      temp_offset = TEMP_OFFSET;
    }
    if(features & FEATURE_SCD30)
    {
      float offset;
      offset = scd30.getTemperatureOffset();
      if((offset == 0) || (offset > 12))
      {
        scd30.setTemperatureOffset(temp_offset); //temperature offset
      }
    }
    else if(features & FEATURE_SCD4X)
    {
      float offset;
      scd4x.getTemperatureOffset(offset);
      if((offset == 0) || (offset > 12))
      {
        scd4x.setTemperatureOffset(temp_offset); //temperature offset
      }
    }
  }
  ws2812.setBrightness(settings.brightness); //0...255

  //USB connection
  if(USBDevice.connected()) //(Serial) uses flow control for detection
  {
    features |= FEATURE_USB;
    delay(1500); //wait 1500ms
    Serial.println("\nCO2 Ampel v" VERSION);
    Serial.print("Features:");
    if(features & FEATURE_SCD30)    { Serial.print(" SCD30"); }
    if(features & FEATURE_SCD4X)    { Serial.print(" SCD4X"); }
    if(features & FEATURE_LPS22HB)  { Serial.print(" LPS22HB"); }
    if(features & FEATURE_BMP280)   { Serial.print(" BMP280"); }
    if(features & FEATURE_WINC1500) { Serial.print(" WINC1500"); }
    if(features & FEATURE_SSD1306)  { Serial.print(" SSD1306"); }
    Serial.println("\n");
  }

  //service menu
  if(run_menu)
  {
    menu(); //open menu
  }

  //Plus version
  if(features & FEATURE_WINC1500)
  {
    if(WiFi.status() != WL_NO_SHIELD) //ATWINC1500 found
    {
      if(wifi_start() != 0) //connect to WiFi network
      {
        if(wifi_start_ap() != 0) //start AP
        {
          features &= ~FEATURE_WINC1500;
        }
      }
      delay(2000); //wait 2s
      if(features & FEATURE_USB)
      {
        String fv = WiFi.firmwareVersion();
        Serial.print("WINC1500 Firmware: ");
        Serial.println(fv);
        byte mac[6];
        WiFi.macAddress(mac);
        Serial.print("MAC: ");
        Serial.print(mac[5], HEX); Serial.print(":"); Serial.print(mac[4], HEX); Serial.print(":"); Serial.print(mac[3], HEX); Serial.print(":");
        Serial.print(mac[2], HEX); Serial.print(":"); Serial.print(mac[1], HEX); Serial.print(":"); Serial.print(mac[0], HEX); Serial.println("");
        IPAddress ip;
        ip = WiFi.localIP();
        Serial.print("IP: "); Serial.println(ip);
        ip = WiFi.subnetMask();
        Serial.print("NM: "); Serial.println(ip);
        ip = WiFi.gatewayIP();
        Serial.print("GW: "); Serial.println(ip);
        Serial.println("");
      }
    }
    else
    {
      features &= ~FEATURE_WINC1500;
      WiFi.end();
    }
  }

  //start measurement
  co2_value = co2_average = START_VALUE;
  if(features & FEATURE_SCD30)
  {
    scd30.setMeasurementInterval(INTERVAL); //set measurement interval
    delay(INTERVAL*1000UL); //wait interval seconds
  }
  else if(features & FEATURE_SCD4X)
  {
    //interval 5s
  }
  else
  {
    if(features & FEATURE_USB)
    {
      Serial.println("Error: CO2 sensor not found");
    }
    leds(COLOR_RED);
    status_led(1000); //status LED
    leds(COLOR_OFF);
    co2_value = co2_average = START_RED;
  }

  return;
}


void ampel(unsigned int co2)
{
  static unsigned int blink=0;

  //LEDs
  if(co2 < settings.range[0]) //blue
  {
    blink = 0;
    leds(COLOR_BLUE);
  }
  else if(co2 < settings.range[1]) //green
  {
    blink = 0;
    leds(COLOR_GREEN);
  }
  else if(co2 < settings.range[2]) //yellow
  {
    blink = 0;
    leds(COLOR_YELLOW);
  }
  else if(co2 < settings.range[3]) //red
  {
    blink = 0;
    leds(COLOR_RED);
  }
  else //red blinking
  {
    if(blink == 0)
    {
      leds(ws2812.Color(10,0,0)); //red at low brightness
    }
    else
    {
      leds(COLOR_RED); //red
    }
    blink = 1-blink; //invert
  }

  //buzzer
  if(co2 < settings.range[4])
  {
    buzzer(0); //buzzer off
  }
  else
  {
    if((blink == 0) && (buzzer_timer == 0) && settings.buzzer)
    {
      buzzer(1); //buzzer on
    }
    else
    {
      buzzer(0); //buzzer off
    }
  }

  return;
}


void loop()
{
  static unsigned int dark=0, sw=0;
  static unsigned long t_switch=0, t_ampel=0, t_light=~((LIGHT_INTERVAL*1000UL*60UL)-60000UL); //check light sensor after 60s
  unsigned int overwrite=0;

  //process serial commands
  serial_service();

  //process WiFi data
  webserver_service();

  //check button
  if(digitalRead(PIN_SWITCH) == LOW) //button pressed
  {
    if(sw == 0)
    {
      sw = 1;
      t_switch = millis(); //save time
    }
  }
  else if(sw != 0) //button released
  {
    sw = 0;
    buzzer(0); //buzzer off
    buzzer_timer = BUZZER_DELAY; //buzzer startup delay
    if((millis()-t_switch) > 3000) //3s button press
    {
      if(features & FEATURE_WINC1500)
      {
        leds(COLOR_VIOLET); //LEDs violet
        wifi_start_ap();
      }
    }
    else if((millis()-t_switch) > 100) //100ms button press
    {
      //cycle: BRIGHTNESS -> /2 -> /4 -> ... -> off -> BRIGHTNESS
      if(settings.brightness == 0)
      {
        settings.brightness = BRIGHTNESS;
      }
      else
      {
        settings.brightness = settings.brightness/2; //halve brightness
        if(settings.brightness < BRIGHTNESS_DARK)
        {
          settings.brightness = 0; //next press will restore BRIGHTNESS
        }
      }
      ws2812.setBrightness(settings.brightness);
      overwrite = 1;
    }
  }

  if((millis()-t_ampel) > 1000) //run ampel logic only once a second
  {
    t_ampel = millis(); //save time

    if(buzzer_timer > 0)
    {
      buzzer_timer--;
    }

    //USB connection
    if(USBDevice.connected()) //(Serial) uses flow control for detection
    {
      features |= FEATURE_USB;
    }
    //else
    //{
    //  features &= ~FEATURE_USB;
    //}

    //read sensor data
    if(check_sensors())
    {
      show_data();
      if(dark == 0)
      {
        status_led(2); //status LED
      }
    }

    co2_average = (co2_average + co2_sensor()) / 2; //recalculate every second
  }
  else if(overwrite == 0)
  {
    return;
  }

  //ampel
  if(remote_on == 0)
  {
    #if AMPEL_AVERAGE > 0
      ampel(co2_average);
    #else
      ampel(co2_value);
    #endif
  }

  //light sensor
  if(remote_on == 0)
  {
    if((millis()-t_light) > (LIGHT_INTERVAL*1000UL*60UL))
    {
      t_light = millis(); //save time

      light_value = light_sensor();
      if(light_value < LIGHT_DARK)
      {
        if(dark == 0)
        {
          dark = 1;
          if(settings.brightness > BRIGHTNESS_DARK)
          {
            ws2812.setBrightness(BRIGHTNESS_DARK); //dark
          }
        }
      }
      else
      {
        if(dark == 1)
        {
          dark = 0;
          ws2812.setBrightness(settings.brightness); //bright
        }
      }
    }
  }

  return;
}
