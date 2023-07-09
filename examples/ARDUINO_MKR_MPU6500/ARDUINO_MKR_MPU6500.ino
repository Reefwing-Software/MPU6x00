/******************************************************************
  @file       ARDUINO_MKR_MPU6500.ino
  @brief      Targeting Arduino MKR boards and the MPU6500 connected
              using SPI - displays raw Gyro counts.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        07/07/23

  1.0.0     Original Release.       07/07/23

******************************************************************/

#include <ReefwingMPU6x00.h>

static const uint8_t MOSI_PIN = 8;
static const uint8_t MISO_PIN = 10;
static const uint8_t SCLK_PIN = 9;
static const uint8_t CS_PIN  = 7;
static const uint8_t INT_PIN = 1;
static const uint8_t LED0_PIN = A4;
static const uint8_t LED1_PIN = A5;

static MPU6500 imu = MPU6500(SPI, CS_PIN);

static void blinkLED(void) {
    const auto msec = millis();
    static uint32_t prev;

    if (msec - prev > 500) {
        static bool on;

        digitalWrite(LED0_PIN, on);
        on = !on;
        prev = msec;
    }
}

static bool gotInterrupt;

static void handleInterrupt(void) {
    gotInterrupt = true;
}

void setup(void) {
    //  Pin Configuration
    pinMode(INT_PIN, INPUT);
    pinMode(LED0_PIN, OUTPUT);
    pinMode(LED1_PIN, OUTPUT);

    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    // Initialise SPI and the MPU6500 IMU
    SPI.begin();

    if (imu.begin()) {
        Serial.println("MPU6500 IMU Connected.");
    }
    else {
        Serial.println("Error initializing IMU.");
        digitalWrite(LED1_PIN, HIGH);
        while(1);
    }

    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop(void) {
    blinkLED();

    if (gotInterrupt) {
        imu.readSensor();

        Serial.print(imu.getRawGyroX());
        Serial.print("  ");
        Serial.print(imu.getRawGyroY());
        Serial.print("  ");
        Serial.print(imu.getRawGyroZ());
        Serial.println(); 

        gotInterrupt = false;
    }
}