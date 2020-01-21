/***************************************************************************
  This sketch makes a Fine Offset WH1080 compatible remote sensor unit out
  of an ESP32, a professional Thies anemometer and a BME280 sensor board.
  Refer to http://blixemboschweer.nl/ for more information on the used hardware.

  This sketch would not have been posible without the help of people who
  shared their code and knowledge on the internet. For example:
  * http://www.susa.net/wordpress/2012/08/raspberry-pi-reading-wh1081-weather-sensors-using-an-rfm01-and-rfm12b/
  * https://www.sevenwatt.com/main/wh1080-protocol-v2-fsk/
  * https://github.com/rinie/LaCrosseITPlusReader
  * The people at Adafruit for their BME280 library.
  * https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
  * https://blog.blinkenlight.net/experiments/dcf77/dcf77-receiver-modules/
  * https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
  * https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide
  * and many others.

  Written by Tsjakka from The Netherlands.
  MIT license, this comment block must be included in any redistribution.
 ***************************************************************************/
#include <DCF77.h>
#include <vfs_api.h>
#include <FSImpl.h>
#include <FS.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <Time.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <HTTP_Method.h>
//#include <ESPmDNS.h>
#include <Update.h>
#include <Adafruit_BME280.h>
//#include <BH1750.h>
#include "WH1080.h"
#include "Thies.h"

Preferences preferences;
WebServer server(80);
WH1080 wh1080;          // The class that takes care of communicating with the base station.
Thies thies;            // The class that collects information from the Thies anemometer.
Adafruit_BME280 bme;    // The class that controls the BME280 sensor.
//BH1750 lightMeter;      // The class that controls the BH1750 sensor. Disabled (the WH1080 does not support this).
WH1080::WeatherData weatherData;    // Data structure for passing weather data to WH1080 class.

const char* host      = "WeatherStation";       // Host name used for Preferences and DNS
const char* ssid      = "<MyNetworkName>";      // Network name
const char* password  = "<MyPassword>";         // Network password
const uint32_t connectPeriod = 90;              // Max period (in s) for setting up a wifi connection
const uint32_t rebootIntoWifiPeriod = 20000;    // If rebooted within this period (in ms) after startup, the system will start in OTA update mode

uint16_t numBoots = 0;
uint8_t startWifi = 0;
bool webServerRunning = false;
bool testMode = false;              // Set to true to run only unit tests

// Timer interrupt declarations
volatile uint32_t timerInterruptCounter = 0;
volatile uint32_t currentIsrAt = 0;
volatile uint32_t previousTransmitAt = 0;

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
const uint32_t LOOP_PERIOD = 48000;
volatile SemaphoreHandle_t timerSemaphore;

// Handle timer interrupts (every 0.25s)
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  timerInterruptCounter++;
  currentIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);

  // Release the semaphore that we check for in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

const int DIRECTION_DIVIDER = 4;    // Every x loops do we sample the direction
int windDirDivider = DIRECTION_DIVIDER;

bool timeSetByDcf = false;          // True if the time has been set by the DCF receiver
bool timeTransmitted = false;       // True if the time has been sent to the base station at least once

void HandleSerialPort(char c) {
  static unsigned long value;
  unsigned char buf[20];

  if ('0' <= c && c <= '9') {
    value = 10 * value + c - '0';
  }
  else if ('a' <= c && c <= 'z') {
    switch (c) {
    case 'i':
      wh1080.initializeRfm();
      break;
    case 'p':
      wh1080.printAllRfmRegs();
      break;
    case 'r':
      wh1080.resetRfm();
      break;
    case 's':
      wh1080.sensorDataUnitTest();
      break;
    case 't':
      wh1080.timeDataUnitTest();
      break;
    case 'q':
      Serial.println("Rebooting");
      delay(3000);
      ESP.restart();
      break;
    }
    value = 0;
  }
}

//#######################################################################################
// ArduinoOTA Login Page

const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='submit')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
//#######################################################################################
// Upload Page
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

//#######################################################################################
// Setup and Loop
 
void setup() {
  Serial.begin(115200);

  // Initialize EEPROM with predefined size
  preferences.begin(host);
  
  // Read from flash memory
  numBoots = preferences.getUShort("numBoots", 0);
  startWifi = preferences.getUChar("startWifi", 0);
  Serial.print("Boot #");
  Serial.println(++numBoots);
  preferences.putUShort("numBoots", numBoots);

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // If the right value was set in EEPROM, start WiFi and wait for the software to be updated.
  // Otherwise, start in weather station mode. This solution was chosen because several GPIOs
  // are used that conflict with the use of WiFi.
  if (startWifi != 0) {
    // Make sure WiFi will not be started again at the next boot.
    preferences.putUChar("startWifi", 0);

    //  This is here to force the ESP32 to reset WiFi and initialize correctly.
    Serial.println("Starting in OTA update mode");
    Serial.print("WiFi status = ");
    Serial.println(WiFi.getMode());
    WiFi.disconnect(true);
    delay(1000);
    WiFi.mode(WIFI_STA);
    delay(1000);
    Serial.print("WiFi status = ");
    Serial.println(WiFi.getMode());
    // End initialization
      
    // Connect to Wi-Fi network with SSID and password
    Serial.print("Connecting to ");
    Serial.println(ssid);
  
    // Initialize wifi
    WiFi.begin(ssid, password);
  
    // Try to connect for some time and then leave it.
    // Because most of the inputs I use are on ADC2, they cannot be used at the same time
    // as WiFi. Therefore I will have to make sure the weather station only connects
    // to my router when I want to update the software.
    time_t firstCheckAt = now();
    while ((WiFi.status() != WL_CONNECTED) && (now() < firstCheckAt + connectPeriod)) {
      delay(500);
      Serial.print(".");
    }
  
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected");
      // Use mDNS for host name resolution
  //    if (!MDNS.begin(host)) { //http://esp32.local
  //      Serial.println("Error setting up MDNS responder!");
  //      while (1) {
  //        delay(1000);
  //      }
  //    }
  //    Serial.println("mDNS responder started");
      
      // Return index page, which is stored in serverIndex, on GET
      server.on("/", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", loginIndex);
      });
      server.on("/serverIndex", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", serverIndex);
      });
      
      // Handle uploading firmware file on POST
      server.on("/update", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");

        // Disconnect WiFi. Apparently, this is necessary in order to use ADC2 GPIO after the reboot.
        WiFi.disconnect();
        delay(100);
        WiFi.mode(WIFI_OFF);
        delay(500);
        ESP.restart();
      }, []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
          Serial.printf("Update: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          // Flashing firmware to ESP
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_END) {
          if (Update.end(true)) { // true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          } else {
            Update.printError(Serial);
          }
        }
      });
      
      server.begin();
      webServerRunning = true;
    }
    else {
      WiFi.disconnect();
      Serial.println();
      Serial.println("No wifi connection established, rebooting");
      delay(5000);
      ESP.restart();
    }
  }
  else {
    Serial.println("Starting in weather station mode");

    // Check for the BME280 sensor
    Serial.println(F("Starting BME280 sensor"));
    if (bme.begin(0x76)) {
      // Set BME280 to scenario for weather monitoring
      Serial.println("Setting BME280 to weather station scenario:");
      Serial.println("  Forced mode, 1x temperature / 1x humidity / 1x pressure oversampling, filter off");
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1, // Temperature
                      Adafruit_BME280::SAMPLING_X1, // Pressure
                      Adafruit_BME280::SAMPLING_X1, // Humidity
                      Adafruit_BME280::FILTER_OFF);
    }
    else {
      Serial.println(F("Could not find a valid BME280 sensor, please check wiring."));
    }
  
    //Serial.println(F("BH1750 initialization"));
    //lightMeter.begin();
  
    // Start monitoring the anemometer
    if (!testMode) {
      thies.initialize();
    }
  
    // Start monitoring the rain gauge
    wh1080.initialize();
  
    Serial.println("Setting up timer interrupt for wind speed calculation");
  
    // Use first timer of 4 (counted from zero).
    // Set divider to 40000 for prescaler. At 80 MHz clock speed this means that a trigger is
    // given 2000 times per second (see ESP32 Technical Reference Manual for more info).
    timer = timerBegin(0, 40000, true);
  
    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer, true);
  
    // Set alarm to call onTimer function every 0.25s
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, 500, true);
  
    // Start the first alarm
    timerAlarmEnable(timer);

    // Make sure WiFi will be started at the next boot, if it occurs within rebootIntoWifiPeriod ms.
    preferences.putUChar("startWifi", 8);

    // In test mode we just do a bunch of unit test and stop operations
    if (testMode) {
      delay(7000);
      wh1080.frequencyTest();
      wh1080.sensorDataUnitTest();
      wh1080.timeDataUnitTest();
      thies.unitTest();
      while (true);
    }
  }
  
  Serial.println("Initialization done");
  Serial.println();  
}

// The main loop of this sketch handles collecting weather and time data and sending it to the base station (WH1080).
void loop() {
  int windDirSample;
  
  // When the system is running more than 20s, the user does not want to reboot into OTA mode.
  if (startWifi == 0 && millis() > rebootIntoWifiPeriod) {
    Serial.println("Next boot will be in weather station mode");
    preferences.putUChar("startWifi", 0);
    startWifi = 8;
  }
  
  // Handle the commands from the serial port
  if (Serial.available()) {
    HandleSerialPort(Serial.read());
  }

  if (webServerRunning) {
    // Handle the webserver for OTA
    server.handleClient();
  }
  else {
    // Check the rain bucket for rain
    wh1080.sampleRainGauge();
  }
  
  // If the Timer has fired, start managing the data. When the timer fires is controlled by onTimer().
  // This won't get executed when the webserver is running.
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    uint32_t isrCount = 0, isrTime = 0;

    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = timerInterruptCounter;
    isrTime = currentIsrAt;
    portEXIT_CRITICAL(&timerMux);

    // Every 0.25s we calculate the wind speed and gust
    thies.sampleWindSpeed();

    // The wind direction should be sampled every second
    if (windDirDivider == 1) {
      windDirSample = thies.sampleWindDirection();
      windDirDivider = DIRECTION_DIVIDER;

      // Check the RFM chip too. I had the problem that the settings got lost once in a while.
      if (!wh1080.rfmIsValid()) {
        wh1080.resetRfm();
        wh1080.initializeRfm();
      }
    }
    else {
      windDirDivider--;
    }

    //Wire.begin(BME280_ADDRESS_ALT);

    // Check if we need to calculate and transmit weather data (once every 48s)
    if (isrTime >= previousTransmitAt + LOOP_PERIOD) {
      previousTransmitAt = previousTransmitAt + LOOP_PERIOD;

      // Print timer interrupt count and time
      Serial.println("-------------------------------------------");
      Serial.print("Call to onTimer no. ");
      Serial.print(isrCount);
      Serial.print(" occurred at ");
      Serial.print(isrTime);
      Serial.println(" ms");

      // Start measurement
      bme.takeForcedMeasurement();
      
      // Temperature and pressure
      weatherData.temperature = bme.readTemperature();
      Serial.print(F("Temperature = "));
      Serial.print(weatherData.temperature);
      Serial.println(" *C");

      // The base station has its own barometer, so no need to send pressure
      Serial.print(F("Pressure = "));
      Serial.print(bme.readPressure() / 100.0F);
      Serial.println(" hPa");

      weatherData.humidity = bme.readHumidity();
      Serial.print(F("Humidity = "));
      Serial.print(weatherData.humidity);
      Serial.println(" %");

      // One tip of the bucket equals ~0.3 mm of rain. (Or is it 0.2794? To be calibrated.)
      weatherData.rain = wh1080.readRain();
      Serial.print(F("Rain = "));
      Serial.print(0.3f * weatherData.rain);
      Serial.println(" mm");

      //Wire.begin(0x23);

      // Light level
      //      frame.lux = lightMeter.readLightLevel();
      //      Serial.print("Light: ");
      //      Serial.print(lux);
      //      Serial.println(" lx");
      //      Serial.println();

      // Wind speed
      weatherData.windSpeed = thies.getWindSpeed();
      Serial.print("Wind speed: ");
      Serial.print(weatherData.windSpeed);
      Serial.println(" m/s");

      weatherData.windGust = thies.getWindGust();
      Serial.print("Gust: ");
      Serial.print(weatherData.windGust);
      Serial.println(" m/s");

      // Wind direction
      weatherData.windDir = thies.getWindDirection();
      Serial.print("Direction: ");
      Serial.println(weatherData.windDir);

      // Test code: To calibrate wind direction through the base station,
      // show the direction in the field for wind speed.
      //weatherData.windSpeed = ((float)windDirSample) / 100;

      /*
         On the WH1080, the DCF code is transmitted five times with 48 second intervals
         between 3-6 minutes past a new hour. The sensor data transmission stops in the
         59th minute. Then there are no transmissions for three minutes, apparently to
         be noise free to acquire the DCF77 signal.
         For my system I have decided to send the DCF code only once or twice an hour to
         improve accuracy of the weather data. The reliability does not seem to suffer.
      */

      if (timeSetByDcf) {
        if (minute() == 3 || !timeTransmitted) {
          // Transmit the current time to the base station
          wh1080.transmitTimeData();

          timeTransmitted = true;
        }
        else {
          if (minute() < 3) {
            // Update the internal clock if a new time was received over DCF
            Serial.print("Checking for new time at minute ");
            Serial.println(minute());
            wh1080.updateTime();
          }
          else if (minute() == 4) {
            // Turn off the DCF receiver
            wh1080.disableDcfReceiver();
          }
          else if (minute() == 59) {
            // Turn on the DCF receiver
            wh1080.enableDcfReceiver();
          }

          // Pass all weather data to the WH1080 instance
          wh1080.setSensorData(&weatherData);
  
          // Send the weather data to the base station
          wh1080.transmitSensorData();

          // Reset the gust after each transmit
          thies.resetWindGust();
        }
      }
      else {
        // Pass all weather data to the WH1080 instance
        wh1080.setSensorData(&weatherData);
  
        // Send the weather data to the base station
        wh1080.transmitSensorData();

        // Reset the gust after each transmit
        thies.resetWindGust();

        Serial.println("Reading DCF time");
        timeSetByDcf = wh1080.updateTime();
      }

      // Once a day reset the running sums, so a bad value doesn't propagate over time.
      if (hour() == 23 && minute() == 59 && second() > 11) {
        //wh1080.resetRain();  We don't need to do this, rain is total since startup
        thies.resetVars();
      }
    }
  }
}
