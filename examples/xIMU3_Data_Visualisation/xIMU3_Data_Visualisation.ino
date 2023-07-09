/******************************************************************
  @file       xIMU3_Data_Visualisation.ino
  @brief      Real-time data from MPU6500 IMU, displayed on the x-IMU3 GUI.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        13/04/23

  1.0.0     Original Release.       13/04/23

  Dependency - Requires that the Reefwing_xIMU3_GUI Library is also
               installed. 

******************************************************************/

#include <Reefwing_xIMU3.h>
#include <ReefwingMPU6x00.h>

static const uint8_t MOSI_PIN = 8;
static const uint8_t MISO_PIN = 10;
static const uint8_t SCLK_PIN = 9;
static const uint8_t CS_PIN  = 7;
static const uint8_t INT_PIN = 1;
static const uint8_t LED0_PIN = A4;
static const uint8_t LED1_PIN = A5;

Reefwing_xIMU3 rx;
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
        rx.sendNotification("MPU6500 IMU Connected.");
    }
    else {
        rx.sendError("MPU6500 IMU Not Connected.");
        digitalWrite(LED1_PIN, HIGH);
        while(1);
    }

    attachInterrupt(INT_PIN, handleInterrupt, RISING);

    //  Calibrate IMU for Bias Offset
    rx.sendNotification("Calibrating IMU - no movement please!");
    imu.calibrateAccelGyro();
    rx.sendNotification("IMU Calibrated.");
}

void loop(void) {
    blinkLED();

    if (gotInterrupt) {
        imu.readSensor();

        //  Read IMU Sensor Data in xIMU3 format
        InertialMessage msg = imu.getInertial();
        TempData td = imu.getTempData();

        //  Send data messages to xIMU3 GUI
        rx.sendInertial(msg);
        rx.sendTemperature(td);

        gotInterrupt = false;
    }
}
