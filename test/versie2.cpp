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
#define MANUAL_PIN 35 // Switch on blower manually (number 1 on switch)
#define AUTO_PIN 34   // Switch on auto mode (number 2 on switch)
#define PRESSURE_SDA_PIN 21
#define PRESSURE_SCL_PIN 22

class RelayController {
private:
    int redStatus;
    int yellowStatus;
    int greenStatus;
    int buzzerStatus;
    int blowerStatus;
    int laserStatus[3];
    int manualStatus;
    int autoStatus;
    float pressure;
    DFRobot_LWLP lwlp;

public:
    const char* ssid;
    const char* password;

    RelayController() {
        ssid = "Cercuits_2_4_GHz"; // Initialize ssid
        password = "Banaanplantage"; // Initialize password
    }

    void readInputs() {
        laserStatus[0] = !digitalRead(LASER_PIN_1); // Read the status of Laser 1
        laserStatus[1] = !digitalRead(LASER_PIN_2); // Read the status of Laser 2
        laserStatus[2] = !digitalRead(LASER_PIN_3); // Read the status of Laser 3
        manualStatus = digitalRead(MANUAL_PIN);
        autoStatus = digitalRead(AUTO_PIN);
        DFRobot_LWLP::sLwlp_t data = lwlp.getData();
        pressure = data.presure;
    }

    void controlOutputs() {
        // Control lamp outputs
        digitalWrite(RED_PIN, redStatus);
        digitalWrite(YELLOW_PIN, yellowStatus);
        digitalWrite(GREEN_PIN, greenStatus);
        digitalWrite(BUZZER_PIN, buzzerStatus);

        // Control blower output
        if (blowerStatus == 1 && (millis() / 500) % 2 == 0) {
            digitalWrite(BLOWER_PIN, HIGH);
        } else {
            digitalWrite(BLOWER_PIN, LOW);
        }
    }

    void setRedStatus(int status) {
        redStatus = status;
    }

    void setYellowStatus(int status) {
        yellowStatus = status;
    }

    void setGreenStatus(int status) {
        greenStatus = status;
    }

    void setBuzzerStatus(int status) {
        buzzerStatus = status;
    }

    void setBlowerStatus(int status) {
        blowerStatus = status;
    }

    void setLaserStatus(int index, int status) {
        if (index >= 0 && index < 3) {
            laserStatus[index] = status;
        }
    }

    int getManualStatus() {
        return manualStatus;
    }

    int getAutoStatus() {
        return autoStatus;
    }

    float getPressure() {
        return pressure;
    }
};

RelayController relayController;

void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(relayController.ssid, relayController.password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
}

void lamp(int lampOption) {
    switch (lampOption) {
        case 1:
            relayController.setRedStatus(0);
            relayController.setYellowStatus(0);
            relayController.setGreenStatus(0);
            break;

        case 2:
            unsigned long startTime = millis();
            unsigned long elapsedTime = 0;
            while (elapsedTime < 3000) {
                digitalWrite(RED_PIN, HIGH);
                delay(500);
                digitalWrite(RED_PIN, LOW);
                delay(500);
                elapsedTime = millis() - startTime;
            }
            break;

        case 3:
            if (relayController.getPressure() > 500) {
                relayController.setYellowStatus(HIGH);
            } else {
                relayController.setYellowStatus(LOW);
            }
            break;

        case 4:
            if (relayController.getBlowerStatus() == 1) {
                relayController.setGreenStatus(HIGH);
            } else {
                relayController.setGreenStatus(LOW);
            }
            break;
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
    relayController.setRedStatus(LOW);
    relayController.setYellowStatus(LOW);
    relayController.setGreenStatus(LOW);
    relayController.setBuzzerStatus(LOW);
    relayController.setBlowerStatus(LOW);

    initWiFi();
    configTime(0, 0, "pool.ntp.org");

    // Start Serial communication
    Serial.begin(9600);
}

void loop() {
    relayController.readInputs();

    if (relayController.getAutoStatus() == HIGH) {
        if (relayController.getBlowerStatus() == LOW && millis() % 2000 < 1000) {
            relayController.setRedStatus(HIGH);
        } else {
            relayController.setRedStatus(LOW);
        }
    }

    if (relayController.getManualStatus() == HIGH) {
        relayController.setBlowerStatus(HIGH);
        relayController.setGreenStatus(HIGH);
    } else {
        relayController.setBlowerStatus(LOW);
        relayController.setGreenStatus(LOW);
    }

    lamp(3); // Invoke lamp function with option 3

    if (relayController.getManualStatus() == LOW && relayController.getAutoStatus() == LOW) {
        relayController.setRedStatus(LOW);
    }

    relayController.controlOutputs();

    delay(10); // Small delay for stability
}
