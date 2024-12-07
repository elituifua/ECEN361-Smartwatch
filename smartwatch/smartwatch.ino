//  ESP8266_SmartWatch.ino
//  Libraries needed:
//  Time.h & TimeLib.h:  https://github.com/PaulStoffregen/Time
//  Timezone.h: https://github.com/JChristensen/Timezone
//  SSD1306.h & SSD1306Wire.h:  https://github.com/squix78/esp8266-oled-ssd1306
//  NTPClient.h: https://github.com/arduino-libraries/NTPClient
//  ESP8266WiFi.h & WifiUDP.h: https://github.com/ekstrand/ESP8266wifi
//  PulseSensor Playground: https://github.com/WorldFamousElectronics/PulseSensorPlayground
//  ESP8266TimerInterrupt: https://github.com/khoih-prog/ESP8266TimerInterrupt
//  Download latest Blynk library here: https://github.com/blynkkk/blynk-library/releases/latest

// 128x64 OLED pinout:
// GND goes to ground
// Vin goes to 3.3V
// Data to I2C SDA (GPIO 4)
// Clk to I2C SCL (GPIO 5)

#include <ESP8266WiFi.h>
#include <WifiUDP.h>
#include <String.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <NTPClient.h>
#include <Time.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <PulseSensorPlayground.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266_ISR_Timer.hpp>
#include <SSD1306I2C.h>
#include <SPI.h>
#include <SparkFunLSM6DS3.h>

#define USE_ARDUINO_INTERRUPTS true
#define BLYNK_PRINT Serial
#define CLEAR_STEP true
#define NOT_CLEAR_STEP false
#define BLYNK_TEMPLATE_ID "TMPL28Z7Rcxq-"
#define BLYNK_TEMPLATE_NAME "Templatezord"

// Create PulseSensorPlayground object
PulseSensorPlayground pulseSensor;

//Create a instance of class LSM6DS3
LSM6DS3Core myIMU(I2C_MODE, 0x6B);

const int PULSE_SENSOR_PIN = 1;
const int LED_PIN = 3;
const int THRESHOLD = 550;  // Threshold for detecting a heartbeat
const int Button1 = 14;
int Button2 = 12;
int Button3 = 13;

int Relay1Pin = 2;  //Relay pin on the other ESP8266
int Relay2Pin = 0;  //Relay pin on the other ESP8266

int Relay1State = HIGH;  // the current state of the output pin
int Relay2State = HIGH;  // the current state of the output pin

String RlSt = String(Relay1State, HEX);

int Relay1ButtonState;  // the current reading from the input pin
int Relay2ButtonState;  // the current reading from the input pin

int lastButtonState1 = LOW;  // the previous reading from the input pin
int lastButtonState2 = LOW;  // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime1 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime2 = 0;  // the last time the output pin was toggled

unsigned long debounceDelay1 = 50;  // the debounce time; increase if the output flickers
unsigned long debounceDelay2 = 50;  // the debounce time; increase if the output flickers

char auth[] = "XXXXXXXXXXXXXXXXXXXXXXXXXXX";  //Enter your Authentication Token
// Define NTP properties
#define NTP_OFFSET 60 * 60             // In seconds
#define NTP_INTERVAL 60 * 1000         // In miliseconds
#define NTP_ADDRESS "ir.pool.ntp.org"  // change this to whatever pool is closest (see ntp.org)

// Set up the NTP UDP client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

// Create a display object
SSD1306Wire display(0x3C, 4, 5);  //0x3d for the Adafruit 1.3" OLED, 0x3C being the usual address of the OLED

const char* ssid = "BYUI_Visitor";  // insert your own ssid
const char* password = "";          // and password
String date;
String t;
String tempC;
int temp;
const char* days[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
const char* months[] = { "Jan", "Feb", "Mar", "Apr", "May", "June", "July", "Aug", "Sep", "Oct", "Nov", "Dec" };
const char* ampm[] = { "AM", "PM" };

const char hostname[] = "api.tomorrow.io";
const String latitude = "43.825386";
const String longitude = "-111.792824";
const String fields = "temperature";
const String apiKey = "0MCI6ym3qe2E6wXZJqW7g5yqddRehDvA";  // API key
const String url = "/v4/timelines?location=" + latitude + "," + longitude + "&fields=" + fields + "&timesteps=current&units=metric&apikey=" + apiKey;

const int port = 80;

unsigned long timeout = 10000;  //ms

WiFiClient client;

BlynkTimer timer;

WidgetBridge bridge1(V1);  // Connect the Relay module


BLYNK_CONNECTED() {
  // Place the AuthToken of the second hardware here
  bridge1.setAuthToken("XXXXXXXXXXXXXXXXXXXXXXXXXXXXX");  // Enter the Auth token of the relay module(Other Esp8266 Module);
}

void setup() {
  Serial.begin(115200);  // most ESP-01's use 115200 but this could vary
  timeClient.begin();    // Start the NTP UDP client

  pulseSensor.analogInput(PULSE_SENSOR_PIN);
  pulseSensor.blinkOnPulse(LED_PIN);
  pulseSensor.setThreshold(THRESHOLD);

  // Check if PulseSensor is initialized
  if (pulseSensor.begin()) {
    Serial.println("PulseSensor object created successfully!");
  }

  // Pedometer Setup
  SetupPedometer();

  Wire.pins(4, 5);   // Start the OLED with GPIO 4 and 5 on ESP-01
  Wire.begin(4, 5);  // 4=sda, 5=scl
  display.init();
  display.flipScreenVertically();

  Blynk.begin(auth, ssid, password, "blynk-cloud.com", 8080);
  // Connect to wifi
  pinMode(Button1, INPUT);
  pinMode(Button2, INPUT);
  pinMode(Button3, INPUT);

  Serial.println("");
  display.drawString(0, 0, "Connected to WiFi.");
  Serial.print(WiFi.localIP());
  Serial.println("");
  display.drawString(0, 24, "Welcome!");
  display.display();
  delay(1000);
}

void loop() {
  // Pedometer
  if (digitalRead(Button2 == LOW)) {
    DisplayPedometer();
  }

  //--------------------------------------------------------------------------//
  // Pulse Reader
  if (digitalRead(Button3 == LOW)) {
    DisplayHeartBeat();
  }

  //--------------------------------------------------------------------------//
  // Weather
  if (digitalRead(Button1 == LOW)) {
    Serial.print("Button pressed");
    GetWeatherData();
    tellTime();
    delay(6000);
  } else {
    display.clear();
  }

  Blynk.run();
  timer.run();
  ControlRelays();

  display.display();
}

//Setup pedometer
void SetupPedometer() {
  //Call .beginCore() to configure the IMU
  if (myIMU.beginCore() != 0) {
    Serial.print("Error at beginCore().\n");
  } else {
    Serial.print("\nbeginCore() passed.\n");
  }

  //Error accumulation variable
  uint8_t errorAccumulator = 0;

  uint8_t dataToWrite = 0;  //Temporary variable

  //Setup the accelerometer******************************
  dataToWrite = 0;  //Start Fresh!
  //  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

  // //Now, write the patched together data
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //Set the ODR bit
  errorAccumulator += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);


  // Enable embedded functions -- ALSO clears the pdeo step count
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
  // Enable pedometer algorithm
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);
  // Step Detector interrupt driven to INT1 pin
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);

  if (errorAccumulator) {
    Serial.println("Problem configuring the device.");
  } else {
    Serial.println("Device O.K.");
  }
  delay(200);
}


// Display Pedometer
void DisplayPedometer() {
  uint8_t readDataByte = 0;
  uint16_t stepsTaken = 0;
  //Read the 16bit value by two 8bit operations
  myIMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  stepsTaken = ((uint16_t)readDataByte) << 8;

  myIMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepsTaken |= readDataByte;

  //Display steps taken
  Serial.print("Steps taken: ");
  Serial.println(stepsTaken);

  display.drawString(0, 0, "Current Steps: " + String(stepsTaken));
}


void DisplayHeartBeat() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_24);

  int currentBPM = pulseSensor.getBeatsPerMinute();

  // Check if a heartbeat is detected
  if (pulseSensor.sawStartOfBeat()) {
    Serial.println("♥ A HeartBeat Happened!");
    Serial.print("BPM: ");
    Serial.println(currentBPM);

    display.drawString(0, 0, "BPM: " + String(currentBPM));
  }

  else {
    display.drawString(0, 0, "Could not read heart beat");  // if sensor fails
    display.display();
  }
}


void ControlRelays() {
  // read the state of the switch into a local variable:
  int reading1 = digitalRead(Relay1ButtonState);
  int reading2 = digitalRead(Relay2ButtonState);

  if (reading1 == LOW || reading2 == LOW) {  // Tell the state of the Lights
    display.drawRect(0, 20, 60, 40);
    display.drawRect(61, 20, 60, 40);
    display.setFont(ArialMT_Plain_10);
    display.drawString(17, 3, "Lights");
    display.drawString(84, 3, "A/C");

    if (Relay1State == HIGH) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(18, 30, "ON");
    } else if (Relay1State == LOW) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(15, 30, "OFF");
    }
    if (Relay2State == HIGH) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(78, 30, "ON");
    } else if (Relay2State == LOW) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(76, 30, "OFF");
    }
  }
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading1 != lastButtonState1) {
    // reset the debouncing timer
    lastDebounceTime1 = millis();
  }
  if (reading2 != lastButtonState2) {
    // reset the debouncing timer
    lastDebounceTime2 = millis();
  }


  if ((millis() - lastDebounceTime1) > debounceDelay1) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading1 != Relay1ButtonState) {
      Relay1ButtonState = reading1;

      // only toggle the LED if the new button state is HIGH
      if (Relay1ButtonState == HIGH) {
        Relay1State = !Relay1State;
      }
    }
  }
  if ((millis() - lastDebounceTime2) > debounceDelay2) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading2 != Relay2ButtonState) {
      Relay2ButtonState = reading2;

      // only toggle the LED if the new button state is HIGH
      if (Relay2ButtonState == HIGH) {
        Relay2State = !Relay2State;
      }
    }
  }
  // set the LED:
  bridge1.digitalWrite(Relay1Pin, Relay1State);
  bridge1.digitalWrite(Relay2Pin, Relay2State);
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState1 = reading1;
  lastButtonState2 = reading2;
}


void tellTime() {
  if (WiFi.status() == WL_CONNECTED)  //Check WiFi connection status
  {
    date = "";  // clear the variables
    t = "";

    // update the NTP client and get the UNIX UTC timestamp
    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();

    // convert received time stamp to time_t object
    time_t local, utc;
    utc = epochTime;

    // Then convert the UTC UNIX timestamp to local time
    TimeChangeRule usEDT = { "EDT", Second, Sun, Mar, 2, +150 };  //UTC - 5 hours - change this as needed
    TimeChangeRule usEST = { "EST", First, Sun, Nov, 2, +150 };   //UTC - 6 hours - change this as needed
    Timezone usEastern(usEDT, usEST);
    local = usEastern.toLocal(utc);

    // now format the Time variables into strings with proper names for month, day etc
    date += days[weekday(local) - 1];
    date += ", ";
    date += months[month(local) - 1];
    date += " ";
    date += day(local);
    date += ", ";
    date += year(local);

    // format the time to 12-hour format with AM/PM and no seconds
    t += hourFormat12(local);
    t += ":";
    if (minute(local) < 10)  // add a zero if minute is under 10
      t += "0";
    t += minute(local);
    t += " ";
    t += ampm[isPM(local)];

    // Display the date and time
    Serial.println("");
    Serial.print("Local date: ");
    Serial.print(date);
    Serial.println("");
    Serial.print("Local time: ");
    Serial.print(t);

    // print the date and time on the OLED
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_24);
    display.drawStringMaxWidth(64, 14, 128, t);  // print time on the display
    display.setFont(ArialMT_Plain_10);
    display.drawStringMaxWidth(64, 42, 128, date);  // print date on the display

    display.drawString(70, 0, "Temp:");  // prints the Temperature from GetWeatherData() function
    display.drawString(100, 0, tempC);   // Replace with "temp" to get temperaute in Farenheit
    display.drawString(113, 0, "C");
    display.display();
  } else  // attempt to connect to wifi again if disconnected
  {
    display.clear();
    display.drawString(0, 18, "Connecting to Wifi...");
    display.display();
    WiFi.begin(ssid, password);
    display.drawString(0, 32, "Connected.");
    display.display();
  }
}
void GetWeatherData() {
  const char* apiKey = "0MCI6ym3qe2E6wXZJqW7g5yqddRehDvA";  // Tomorrow.io API Key
  const String latitude = "43.825386";                      // Latitude
  const String longitude = "-111.792824";                   // Longitude
  const String fields = "temperature";
  const String tomorrowURL = "/v4/timelines?location=" + latitude + "," + longitude + "&fields=" + fields + "&timesteps=current&units=metric&apikey=" + apiKey;

  Serial.print("Connecting to ");
  Serial.println(hostname);

  if (!client.connect(hostname, port)) {
    Serial.println("Connection failed");
    return;
  }

  // Send GET request
  String request = "GET " + tomorrowURL + " HTTP/1.1\r\n" + "Host: " + hostname + "\r\n" + "Connection: close\r\n\r\n";
  client.print(request);

  // Wait for the response
  unsigned long timestamp = millis();
  while (!client.available() && (millis() - timestamp < timeout)) {
    delay(1);
  }

  // Read and parse the response
  String response = "";
  while (client.available()) {
    response += client.readString();
  }
  client.stop();

  // Find the temperature in the response
  int tempIndex = response.indexOf("\"temperature\":");
  if (tempIndex != -1) {
    int startIndex = response.indexOf(":", tempIndex) + 1;
    int endIndex = response.indexOf(",", startIndex);
    tempC = response.substring(startIndex, endIndex);
    Serial.print("Local temperature: ");
    Serial.print(tempC);
    Serial.println("°C");
  } else {
    Serial.println("Temperature data not found in the response");
  }


  // Send GET request
  String req = "GET " + url + " HTTP/1.1\r\n" + "Host: " + hostname + "\r\n" + "Connection: close\r\n" + "\r\n";
  client.print(req);

  // Wait for response from server
  delay(500);
  timestamp = millis();
  while (!client.available() && (millis() < timestamp + timeout)) {
    delay(1);
  }

  // Parse temperature
  if (client.find("temp\":")) {
    temp = client.parseInt();
    tempC = (temp - 32) * 5 / 9;
    Serial.print("Local temperature: ");
    Serial.print(tempC);
    Serial.println("°C");
  }

  // Flush receive buffer
  while (client.available()) {
    client.readStringUntil('\r');
  }

  // Close TCP connection
  client.stop();
  Serial.println();
  Serial.println("Connection closed");
}