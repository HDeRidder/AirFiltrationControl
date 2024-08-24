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
#define MANUAL_PIN 35 //switch on blower manually (number 1 on switch)
#define AUTO_PIN 34   //switch on auto mode (number 2 on switch)
#define PRESSURE_SDA_PIN 21
#define PRESSURE_SCL_PIN 22

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

// Wifi configuration
const char *ssid = "Cercuits_2_4_GHz";
const char *password = "Banaanplantage";

//Functions

//Read all inputs
void Digital_IN()
{
    laserStatus[0] = !digitalRead(LASER_PIN_1); // Read the status of Laser 1
    laserStatus[1] = !digitalRead(LASER_PIN_2); // Read the status of Laser 2
    laserStatus[2] = !digitalRead(LASER_PIN_3); // Read the status of Laser 3
    manualStatus = digitalRead(MANUAL_PIN);
    autoStatus = digitalRead(AUTO_PIN);
    data = lwlp.getData();
    pressure = data.presure;
}

//Control all outputs
void Digital_OUT()
{
    digitalWrite(YELLOW_PIN, yellowStatus);
    digitalWrite(RED_PIN, redStatus);
    digitalWrite(GREEN_PIN, greenStatus);
    digitalWrite(BUZZER_PIN, buzzerStatus);
    digitalWrite(BLOWER_PIN, blowerStatus);
}

// Initialize WiFi
void initWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
}

// Aansturen van lamp
void lamp(int lampOption)
{
    unsigned long startTime = 0;
    unsigned long elapsedTime = 0;

    switch (lampOption)
    {
    // Turn everything off
    case 1:
        redStatus = 0;
        yellowStatus = 0;
        greenStatus = 0;
        //Serial.println("All lamps are turned off.");
        //Serial.println("Which lamp option do you want to enable?");
        break;

    // Warning signal before turning blower on/off
    case 2:
        startTime = millis();
        elapsedTime = 0;

        while (elapsedTime < 3000)
        { // 3 seconds = 3000 milliseconds
            digitalWrite(RED_PIN, HIGH); // Turn LED on
            delay(500);
            digitalWrite(RED_PIN, LOW); // Turn LED off
            delay(500);

            elapsedTime = millis() - startTime;
        }
        //Serial.println("Warning! Blower is about to turn on.");
        break;

    // Indicate filter bag status
    case 3:
        if (pressure > 500){
            digitalWrite(YELLOW_PIN, HIGH); // Turn on yellow lamp
        }
        else if (pressure < 100){
            digitalWrite(YELLOW_PIN, LOW); // Turn off yellow lamp
        }
        break;

    case 4:
        if (blowerStatus == 1)
        {
            digitalWrite(GREEN_PIN, HIGH);
            //Serial.println("Green lamp is turned on.");
        }
        else
        {
            digitalWrite(GREEN_PIN, LOW);
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

void blower(int laserStatus[])
{
    static unsigned long timerStart = 0; // Variable to store the start time for warning blink

    if (isAnyElementHigh(laserStatus))
    { // If one of the LASER_PINS is high
        digitalWrite(BLOWER_PIN, HIGH); // Turn on the blower
        //Serial.println("The blower is turned on.");
        blowerStatus = 1;
        timerStart = 0; // Reset the timer
    }
    else
    { // If all LASER_PINS are low
        if (timerStart == 0)
        { // If timer is not running
            timerStart = millis(); // Start the timer
            //Serial.println("The laser is turned off. Timer started.");
        }
        else
        {
            if (millis() - timerStart >= 10000)
            { // If 10 seconds have passed
                digitalWrite(BLOWER_PIN, LOW); // Turn off the blower
                blowerStatus = 0;
                //Serial.println("The blower is turned off due to inactivity of the laser.");
                timerStart = -1; // Reset the timer
            }
        }
    }
}

void setup()
{

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
}

void loop()
{

    if (autoStatus == HIGH)
    {
        // digitalWrite(GREEN_PIN, LOW);
        // digitalWrite(YELLOW_PIN, HIGH);
        // digitalWrite(RED_PIN, LOW);
        blower(laserStatus); // Continuously run the blower function

        // if (Serial.available() > 0) { // Check if there is serial input available
        //  int lampOption = Serial.parseInt(); // Read the integer input from Serial Monitor

        // Call the lamp function with the selected option
        //  lamp(lampOption);
    }

    if (manualStatus == HIGH)
    {
        // digitalWrite(GREEN_PIN, HIGH);
        // digitalWrite(YELLOW_PIN, LOW);
        // digitalWrite(RED_PIN, LOW);
        // lamp(2);
        // Switch on the blower and green lamp
        if (blowerStatus == LOW)
        {
            lamp(2);
        }
        digitalWrite(BLOWER_PIN, HIGH);
        blowerStatus = 1;
        lamp(4);
    }

    if (manualStatus == LOW && autoStatus == LOW)
    {
        // digitalWrite(GREEN_PIN, LOW);
        // digitalWrite(YELLOW_PIN, LOW);
        // digitalWrite(RED_PIN, HIGH);
        // Turn off all pins
        lamp(1);
        digitalWrite(BLOWER_PIN, LOW);
        blowerStatus = 0;
    }
}
