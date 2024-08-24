#include <Arduino.h>
#include <WiFi.h>
#include "time.h"
#include <DFRobot_LWLP.h>

// Define the relay pins
#define RED_PIN 2
#define YELLOW_PIN 4 // for all the other systems
#define GREEN_PIN 5
#define BUZZER_PIN 18
#define BLOWER_PIN 19
#define LASER_PIN_1 13
#define LASER_PIN_2 27
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
//int laser_1_Status[3] = {0, 0, 0};
int manualStatus = 0;
int autoStatus = 0;
const char *ntpServer = "pool.ntp.org"; //
DFRobot_LWLP lwlp; // initialize pressure sensor
DFRobot_LWLP::sLwlp_t data; // initialize data from pressure sensor
float pressure = 0.0;
int potentioValue = 0;
float maxPressure = 0.0;
unsigned long previousMillis;
unsigned long currentMillis;
const long blinkInterval = 500;
const long blinkDuration = 3100;
const long blowerOffDelay = 900000;
const long pressureDuration = 10000;
const long pressureInterval = 600;
int count = 0;
int pressureCount = 0;
int warningFlag = 0;
float pressureArray[256];
uint8_t pressureIndex = 0;
int pressureLength = 256;
//_________________________________________________________________________________________________________________________________________

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
    pressure = abs(data.presure);
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

// Control lamp with different possible cases
void lamp(int lampOption) {
  switch (lampOption) {

  // Turn everything off (at startup)
  case 1:
    redStatus = 0;
    yellowStatus = 0;
    greenStatus = 0;
    break;

  // Warning signal before turning blower on
  case 2:
    static unsigned long blinkTimerStart = 0; 
    blinkTimerStart = millis(); 
    while (millis() - blinkTimerStart <= blinkDuration) {
      buzzerStatus = 1;
      currentMillis = millis();
      if (currentMillis - previousMillis >= blinkInterval && count <= 6){
        previousMillis = currentMillis;
        count += 1;
        redStatus = !redStatus;
        Digital_OUT();
      }
    }
    warningFlag = 0;
    buzzerStatus = 0;
    count = 0;
    break;

  // Indicate that filter bag is full
  case 3:
    static unsigned long pressureTimerStart = 0; 
    pressureTimerStart = millis();
    greenStatus = blowerStatus;
    while (millis() - pressureTimerStart <= pressureDuration) {
      currentMillis = millis();
      Serial.print("pressure count = ");
      Serial.println(pressureCount);
      if (currentMillis - previousMillis >= pressureInterval && pressureCount <= 5){
        previousMillis = currentMillis;
        pressureCount += 1;
        yellowStatus = !yellowStatus;
        Digital_OUT();
      }
    }
    pressureCount = 0;
    break;
  
  // Indicate the status of the blower
  case 4:
    if (blowerStatus == 1) {
        greenStatus = 1;
    } 
    if (blowerStatus == 0 && autoStatus == 0) {
        greenStatus = 0;
    }
    break;

  case 5:
    redStatus = 0;
    break;
  
  case 6: 
    yellowStatus = 0;
    break;

  case 7: 
    greenStatus = 0;
    break;
  
  case 8: // Off mode
    redStatus = 1;
    break;
  
  case 9: // Auto mode 
    redStatus = 1;
    greenStatus = 1;
    break;
  }
}

// Check if at least one of the lasers is active
bool isAnyElementHigh(int laserStatus[])
{
  //to rework add array
    for (int i = 0; i < 3; ++i)
    {
        if (laserStatus[i] == HIGH)
        {
            return true; // Return true if at least one element is HIGH
        }
    }
    return false; // Return false if none of the elements are HIGH
}

// Control the blower
void blower(int laserStatus[]) {
  static unsigned long blowerTimerStart = 0; // Variable to store the start time for warning blink
  
  if (isAnyElementHigh(laserStatus)) { // If one of the LASER_PINS is high
    blowerStatus = 1;
    blowerTimerStart = 0; // Reset the timer
  } else { // If all LASER_PINS are low
    if (blowerTimerStart == 0) { // If timer is not running
      blowerTimerStart = millis(); // Start the timer
      Serial.print("blowertimerstart = ");
      Serial.println(blowerTimerStart);
    } 
    else {
      if ((millis() - blowerTimerStart) >= (blowerOffDelay - blinkDuration) && blowerStatus == 1) { // If blower is about to turn off 
        warningFlag = 1;
      }
      if (millis() - blowerTimerStart >= blowerOffDelay) { // If blowerOffDelay time has passed
        blowerStatus = 0;
        warningFlag = 0;
        blowerTimerStart = -1; // Reset the timer
      }
    }
  }
}

int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

void monitorPressure(float pressure, float maxPressure, int yellowStatus) {

  pressureArray[pressureIndex] = abs(data.presure);
  Serial.print(data.presure);
  
  pressureIndex++;
  Serial.println("Starting median calculation");
  qsort(pressureArray, pressureLength, sizeof(pressureArray[0]), sort_desc);  Serial.print("Pressure = ");
  float median = pressureArray[int(pressureLength/2)];
  Serial.print("Median value = ");
  Serial.println(median);
  Serial.println(pressureIndex);

  if (median > maxPressure ) {
    
    lamp(3); // Turn on the yellow LED
    } 
  if (median < maxPressure && yellowStatus == 1) {
      lamp(1); // Turn off the yellow LED
    }
}

//_____________________________________________________________________________________________________________________________

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
  delay(500);
  // Initially turn off all relays
  digitalWrite(YELLOW_PIN, HIGH);
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(BLOWER_PIN, LOW);
  delay(500);

  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  //int pressureLength = sizeof(pressureArray) / sizeof(float);



  //Start wifi connection
  //initWiFi();
  //configTime(0, 0, ntpServer);

  // Start Serial communication
  Serial.begin(9600);
  int N = 0;
  while (lwlp.begin() != 0 or N<=10) {
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
      N++;
      delay(500);
  }

}

//_____________________________________________________________________________________________________________________________

void loop() {

    currentMillis = millis();
    Digital_IN();
    float maxPressure = floatMap(potentioValue, 0, 4095, 0, 500); // map the measured potentiometer voltage to a corresponding pressure level
    
    // Things to do when the switch is set to auto mode (II)
    if (autoStatus == 1) {
      if (blowerStatus != 1) {
        lamp(9);
      }
        Digital_OUT();
        if (blowerStatus == 0 && isAnyElementHigh(laserStatus)) {
          lamp(2);  
        }
        if (warningFlag == 1 ) {
          lamp(2);  
          warningFlag = 0;
        }
        // Continuously run the blower function
        blower(laserStatus); 
        monitorPressure(pressure, maxPressure, yellowStatus);
        lamp(4);
        }

    // Things to do when the switch is off
    if (manualStatus == 0 && autoStatus == 0) {
      lamp(8);
      Digital_OUT();
      if (blowerStatus == 1 ) {
          lamp(2);  
        }
      blowerStatus = 0;
      lamp(4);
      monitorPressure(pressure, maxPressure, yellowStatus);
    }

    //Things to do when switch is set to manual mode (I)
    if (manualStatus == 1) {
        if (blowerStatus == 0) { 
            lamp(2);
        } 
        lamp(1);
        blowerStatus = 1;
        monitorPressure(pressure, maxPressure, yellowStatus);
        lamp(4);

    }
    Digital_OUT(); 
}