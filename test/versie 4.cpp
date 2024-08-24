#include <Arduino.h>
#include <WiFi.h>
#include "time.h"
#include <DFRobot_LWLP.h>

// Define the relay pins
#define RED_PIN 2
#define YELLOW_PIN 4
#define GREEN_PIN 5
#define BUZZER_PIN 18
#define BLOWER_PIN 19
#define LASER_PIN_1 13
#define LASER_PIN_2 12
#define LASER_PIN_3 14
#define MANUAL_PIN 35 //switch position I
#define AUTO_PIN 34 //switch position II
#define POTENTIO_PIN 32

//#define PRESSURE_SDA_PIN 21
//#define PRESSURE_SCL_PIN 22

// Wifi configuration
const char* ssid     = "Cercuits_2_4_GHz";
const char* password = "Banaanplantage";

//initializing variables
int redStatus = 0;
int yellowStatus = 0;
int greenStatus = 0;
int buzzerStatus = 0;
int blowerStatus = 0;
int laserStatus[3] = {0, 0, 0};
int manualStatus = 0;
int autoStatus = 0;
const char *ntpServer = "pool.ntp.org";
DFRobot_LWLP lwlp;
DFRobot_LWLP::sLwlp_t data;
float pressure = 0.0;
int potentioValue = 0;
float maxPressure = 0.0;
unsigned long previousMillis;
unsigned long currentMillis;
const long interval = 500;
int count = 0;
//Functions

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

// Potentiometer mapping
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Read all inputs
void Digital_IN(){
    laserStatus[0] = !digitalRead(LASER_PIN_1); // Read the status of Laser 1
    laserStatus[1] = !digitalRead(LASER_PIN_2); // Read the status of Laser 2
    laserStatus[2] = !digitalRead(LASER_PIN_3); // Read the status of Laser 3
    manualStatus = digitalRead(MANUAL_PIN);
    autoStatus = digitalRead(AUTO_PIN);
    data = lwlp.getData();
    pressure = data.presure;
    potentioValue = analogRead(POTENTIO_PIN);
}

//Control all outputs
void Digital_OUT(){
    digitalWrite(YELLOW_PIN, yellowStatus);
    digitalWrite(RED_PIN, redStatus);
    digitalWrite(GREEN_PIN, greenStatus);
    digitalWrite(BUZZER_PIN, buzzerStatus);
    digitalWrite(BLOWER_PIN, blowerStatus);
}


// Control lamp
void lamp(int lampOption) {
  switch (lampOption) {

  // Turn everything off at startup
  case 1:
    redStatus = 0;
    yellowStatus = 0;
    greenStatus = 0;
    //digitalWrite(YELLOW_PIN, LOW);
    //digitalWrite(RED_PIN, LOW);
    //digitalWrite(GREEN_PIN, LOW);
    //digitalWrite(BUZZER_PIN, LOW);
    //Serial.println("All lamps and buzzer are turned off.");
    //Serial.println("Which lamp option do you want to enable?");
    break;

  // Warning signal before turning blower on
  case 2:
    count = 0;
    if (currentMillis - previousMillis >= interval && count < 6){
      previousMillis = currentMillis;
      count += 1;
      redStatus = !redStatus;
      Digital_OUT();
      Serial.print("The current count is: ");
      Serial.println(count);
      Serial.print("The current red LED status is: ");
      Serial.println(redStatus);
    }
    break;

  // Indicate that filter bag is full
  case 3:
    
    if (pressure > maxPressure ) {
      //digitalWrite(YELLOW_PIN, HIGH); // Turn on yellow lamp
      yellowStatus = 1;
    } 
    else {
      //digitalWrite(YELLOW_PIN, LOW); // Turn off yellow lamp
      yellowStatus = 0;
    }
    break;

  case 4:
    if (blowerStatus == 1) {
        //digitalWrite(GREEN_PIN, HIGH);
        greenStatus = 1;
        //Serial.println("Green lamp is turned on.");
    } 
    else {
        greenStatus = 0;
        //digitalWrite(GREEN_PIN, LOW);
        //Serial.println("Green lamp is turned off.");
    }
    break;
  }
}

bool isAnyElementHigh(int laserStatus[])
{
    for (int i = 0; i < 3; ++i)
    {
        if (laserStatus[i] == HIGH)
        {
            return true; // Return true if at least one element is HIGH
        }
    }
    return false; // Return false if none of the elements are HIGH
}

void blower(int laserStatus[]) {
  static unsigned long timerStart = 0; // Variable to store the start time for warning blink
  if (isAnyElementHigh(laserStatus)) { // If one of the LASER_PINS is high
    //digitalWrite(BLOWER_PIN, HIGH); // Turn on the blower
    //Serial.println("The blower is turned on.");
    blowerStatus = 1;
    timerStart = 0; // Reset the timer
  } else { // If all LASER_PINS are low
    if (timerStart == 0) { // If timer is not running
      timerStart = millis(); // Start the timer
      //Serial.println("The laser is turned off. Timer started.");
    } else {
      if (millis() - timerStart >= 10000) { // If 10 seconds have passed
        //digitalWrite(BLOWER_PIN, LOW); // Turn off the blower
        blowerStatus = 0;
        //Serial.println("The blower is turned off due to inactivity of the laser.");
        timerStart = -1; // Reset the timer
      }
    }
  }
}


void setup() {
  
  // Initialize relay pins as OUTPUT
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BLOWER_PIN, OUTPUT);
  pinMode(LASER_PIN_1, INPUT);
  pinMode(LASER_PIN_2, INPUT);
  pinMode(LASER_PIN_3, INPUT);
  pinMode(MANUAL_PIN, INPUT);
  pinMode(AUTO_PIN, INPUT);

  // Initially turn off all relays
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BLOWER_PIN, LOW);

  initWiFi();
  configTime(0, 0, ntpServer);

  // Start Serial communication
  Serial.begin(9600);
  // Serial.println("Which lamp option do you want to enable?");
  while (lwlp.begin() != 0) {
  Serial.println("Failed to initialize the chip, please confirm the chip connection");
  delay(1000);
  }

}

void loop() {
    unsigned long currentMillis = millis();
    Digital_IN();
    //Serial.print("Differential Pressure: ");
    //Get pressure difference in unit pa 
    //Serial.print(data.presure);
    //Serial.println(" pa");
    float maxPressure = floatMap(potentioValue, 0, 4095, 0, 500);
    //Serial.print("maxPressure = ");
    //Serial.println(maxPressure);
    //delay(500);
    if (autoStatus == 1) {
        Serial.println("autoStatus enabled");
        // digitalWrite(GREEN_PIN, LOW);
        // digitalWrite(YELLOW_PIN, HIGH);
        // digitalWrite(RED_PIN, LOW);
        if (blowerStatus == 0 && isAnyElementHigh(laserStatus)) {
            Serial.println("Warning! Blower is about to turn on!"); 
            lamp(2);
        }
        else {
        lamp(1); // Turn off the red LED
        }
        // Continuously run the blower function
        blower(laserStatus); 
        if (pressure > maxPressure && yellowStatus == 0) {
            lamp(3);
        } else if (pressure <= maxPressure && yellowStatus == 1) {
            lamp(1); // Turn off the yellow LED
        }
        lamp(4);
        // if (Serial.available() > 0) { // Check if there is serial input available
        //  int lampOption = Serial.parseInt(); // Read the integer input from Serial Monitor

        // Call the lamp function with the selected option
        //  lamp(lampOption); 
        }

    if (manualStatus == 0 && autoStatus == 0) {
        // digitalWrite(GREEN_PIN, LOW);
        // digitalWrite(YELLOW_PIN, LOW);
        // digitalWrite(RED_PIN, HIGH);
        // Turn off all pins
        lamp(1);
        //digitalWrite(BLOWER_PIN, LOW);
        blowerStatus = 0;
    }

    if (manualStatus == 1) {
        // digitalWrite(GREEN_PIN, HIGH);
        // digitalWrite(YELLOW_PIN, LOW);
        // digitalWrite(RED_PIN, LOW);
        // lamp(2);
        // Switch on the blower and green lamp
        if (blowerStatus == 0) { 
            lamp(2);
        } 
        //digitalWrite(BLOWER_PIN, HIGH);
        blowerStatus = 1;
        if (pressure > maxPressure && yellowStatus == 0) {
            lamp(3);
        } else if (pressure <= maxPressure && yellowStatus == 1) {
            lamp(1);
        }
        lamp(4);
    }
    Digital_OUT(); 
}

