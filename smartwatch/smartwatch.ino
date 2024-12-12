// 128x64 OLED pinout:
// GND goes to ground
// Vin goes to 3.3V
// Data to I2C SDA (GPIO 4)
// Clk to I2C SCL (GPIO 5)
// 7, 8, 9, 10, 6, 11

#include <ESP8266WiFi.h>
#include <WifiUDP.h>
#include <SSD1306Wire.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <PulseSensorPlayground.h>
#include <BlynkSimpleEsp8266.h>
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

const int PULSE_SENSOR_PIN = 7;
const int LED_PIN = 8;
const int THRESHOLD = 550;  // Threshold for detecting a heartbeat

// Define pins for buttons
const int button1Pin = 14;  // Button 1: Weather
const int button2Pin = 12;  // Button 2: Pedometer
const int button3Pin = 13;  // Button 3: Pulse reader

// Variables to track state
int currentMode = 0;  // 0 = Time, 1 = Weather, 2 = Pedometer, 3 = Pulse reader

const int relay1Pin = 5;   // Relay 1 pin
const int relay2Pin = 4;   // Relay 2 pin
const int relay3Pin = 16;  // Relay 3 pin

// Variables to store button states and debounce timers
int relay1ButtonState = HIGH;  // Current state of Relay 1 button
int relay2ButtonState = HIGH;  // Current state of Relay 2 button
int relay3ButtonState = HIGH;  // Current state of Relay 3 button

bool relay1State = false;  // Current state of Relay 1
bool relay2State = false;  // Current state of Relay 2
bool relay3State = false;  // Current state of Relay 3

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTimes[3] = { 0, 0, 0 };
int lastButtonStates[3] = { HIGH, HIGH, HIGH };
const unsigned long debounceDelay = 50;  // Debounce delay in milliseconds

char auth[] = "52oHM469YYP-h8m-pRiJWH-sVk-RlXEV"; Authentication Token
// Define NTP properties
#define NTP_OFFSET 60 * 60             // In seconds
#define NTP_INTERVAL 60 * 1000         // In miliseconds
#define NTP_ADDRESS "ir.pool.ntp.org" 

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
  bridge1.setAuthToken("52oHM469YYP-h8m-pRiJWH-sVk-RlXEV");
}


void setup() {
  Serial.begin(115200);  // most ESP-01's use 115200 but this could vary
  timeClient.begin();    // Start the NTP UDP client

  pulseSensor.analogInput(PULSE_SENSOR_PIN);
  pulseSensor.blinkOnPulse(LED_PIN);
  pulseSensor.setThreshold(THRESHOLD);

  // Initialize pins
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);

  pinMode(relay1Pin, OUTPUT);  // Set relay pins as outputs
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);

  // Initialize relays to OFF
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  digitalWrite(relay3Pin, LOW);

  // Check if PulseSensor is initialized
  if (pulseSensor.begin()) {
    Serial.println("PulseSensor object created successfully!");
  }

  // Pedometer Setup
  SetupPedometer();

  // Initialize OLED with default pins for ESP-01 (GPIO 4 and 5)
  Wire.begin(4, 5);  // 4=sda, 5=scl
  display.init();
  display.flipScreenVertically();

  // Now configure I2C for the accelerometer (GPIO 12 and 14)
  Wire.pins(9, 10);   // Assign GPIO 9 and 10 for I2C communication
  Wire.begin();        // Re-initialize I2C with new pins for the accelerometer

  // Connect to wifi
  Blynk.begin(auth, ssid, password, "blynk-cloud.com", 8080);

  display.drawString(0, 0, "Connected to WiFi.");
  Serial.print(WiFi.localIP());
  display.drawString(0, 24, "Welcome!");
  display.display();
  delay(1000);
}


void loop() {
  // Debounce variables for each button
  static int lastRelay1ButtonState = HIGH;
  static unsigned long lastDebounceTime1 = 0;

  static int lastRelay2ButtonState = HIGH;
  static unsigned long lastDebounceTime2 = 0;

  static int lastRelay3ButtonState = HIGH;
  static unsigned long lastDebounceTime3 = 0;

  // Handle Button 1 (Weather)
  if (debounceButton(button1Pin, lastButtonStates[0], lastDebounceTimes[0])) {
    currentMode = (currentMode == 1) ? 0 : 1;  // Toggle between Weather and Time
  }

  // Handle Button 2 (Pedometer)
  if (debounceButton(button2Pin, lastButtonStates[1], lastDebounceTimes[1])) {
    currentMode = (currentMode == 2) ? 0 : 2;  // Toggle between Pedometer and Time
  }

  // Handle Button 3 (Pulse Reader)
  if (debounceButton(button3Pin, lastButtonStates[2], lastDebounceTimes[2])) {
    currentMode = (currentMode == 3) ? 0 : 3;  // Toggle between Pulse Reader and Time
  }

  switch (currentMode) {
    case 0:
      tellTime();
      break;
    case 1:
      Serial.print("Button 1 pressed");
      GetWeatherData();
      break;
    case 2:
      Serial.print("Button 2 pressed");
      DisplayPedometer();
      break;
    case 3:
      Serial.print("Button 3 pressed");
      DisplayHeartBeat();
      break;
  }

  Blynk.run();
  timer.run();
  ControlRelays();
  display.display();
}


bool debounceButton(int buttonPin, int& lastButtonState, unsigned long& lastDebounceTime) {
  int currentButtonState = digitalRead(buttonPin);

  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();  // Reset debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay && currentButtonState == LOW && lastButtonState == HIGH) {
    lastButtonState = currentButtonState;  // Update state
    return true;                           // Button press detected
  }

  lastButtonState = currentButtonState;  // Update state
  return false;                          // No button press detected
}


void SetupPedometer() {
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
  int reading1 = digitalRead(button1Pin);
  int reading2 = digitalRead(button2Pin);
  int reading3 = digitalRead(button3Pin);

  if (reading1 == LOW || reading2 == LOW) {  // Tell the state of the Lights
    display.drawRect(0, 20, 60, 40);
    display.drawRect(61, 20, 60, 40);
    display.setFont(ArialMT_Plain_10);
    display.drawString(17, 3, "Lights");
    display.drawString(84, 3, "A/C");

    if (relay1State == HIGH) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(18, 30, "ON");
    } else if (relay1State == LOW) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(15, 30, "OFF");
    }
    if (relay2State == HIGH) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(78, 30, "ON");
    } else if (relay2State == LOW) {
      display.setFont(ArialMT_Plain_16);
      display.drawString(76, 30, "OFF");
    }
  }
  // check to see if you just pressed the button

  // If the switch changed, due to noise or pressing:
  if (reading1 != lastButtonStates[0]) {
    // reset the debouncing timer
    lastDebounceTimes[0] = millis();
  }
  if (reading2 != lastButtonStates[1]) {
    // reset the debouncing timer
    lastDebounceTimes[1] = millis();
  }
  if (reading3 != lastButtonStates[2]) {
    // reset the debouncing timer
    lastDebounceTimes[2] = millis();
  }

  if ((millis() - lastDebounceTimes[0]) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading1 != relay1ButtonState) {
      relay1ButtonState = reading1;

      // only toggle the LED if the new button state is HIGH
      if (relay1ButtonState == HIGH) {
        relay1State = !relay1State;
      }
    }
  }
  if ((millis() - lastDebounceTimes[1]) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading2 != relay2ButtonState) {
      relay2ButtonState = reading2;

      // only toggle the LED if the new button state is HIGH
      if (relay2ButtonState == HIGH) {
        relay2State = !relay2State;
      }
    }
  }
  if ((millis() - lastDebounceTimes[2]) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading3 != relay3ButtonState) {
      relay3ButtonState = reading3;

      // only toggle the LED if the new button state is HIGH
      if (relay3ButtonState == HIGH) {
        relay3State = !relay3State;
      }
    }
  }
  // set the LED:
  bridge1.digitalWrite(relay1Pin, relay1State);
  bridge1.digitalWrite(relay2Pin, relay2State);
  bridge1.digitalWrite(relay3Pin, relay3State);
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonStates[0] = reading1;
  lastButtonStates[1] = reading2;
  lastButtonStates[2] = reading3;
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
    TimeChangeRule usMDT = { "MDT", Second, Sun, Mar, 2, -360 };  // UTC-6 hours
    TimeChangeRule usMST = { "MST", First, Sun, Nov, 2, -420 };   // UTC-7 hours
    Timezone usMountain(usMDT, usMST);
    local = usMountain.toLocal(utc);

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
  } else {
    unsigned long timeout = millis();
    display.clear();
    display.drawString(0, 18, "Connecting to Wifi...");
    display.display();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - timeout > 10000) {  // 10 seconds timeout
        display.clear();
        display.drawString(0, 32, "Failed to connect");
        display.display();
        return;
      }
    }
    display.clear();
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

  if (!client.available()) {
    Serial.println("Timeout or no data received");
    display.clear();
    display.drawString(0, 32, "Timeout or no data");
    display.display();
    client.stop();
    return;
  }

  // Read and parse the response
  String response = "";
  while (client.available()) {
    response += client.readString();
  }
  client.stop();

  int tempIndex = response.indexOf("\"temperature\":");
  if (tempIndex == -1) {
    Serial.println("Temperature data not found");
    display.clear();
    display.drawString(0, 32, "No Temp Data");
    display.display();
    return;
  }

  // Extract temperature value
  int tempStartIndex = response.indexOf(":", tempIndex) + 1;
  int tempEndIndex = response.indexOf(",", tempStartIndex);

  if (tempEndIndex == -1) {
    tempEndIndex = response.indexOf("}", tempStartIndex);
  }

  String tempStr = response.substring(tempStartIndex, tempEndIndex);
  tempC = tempStr;  // Store temperature for later use

  Serial.print("Current Temperature: ");
  Serial.println(tempStr);

  // Display temperature on OLED screen
  display.clear();
  display.drawString(0, 0, "Weather Data:");
  display.drawString(0, 24, "Temp: " + tempC + "C");
  display.display();
}
