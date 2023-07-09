/******************************************************************
  @file       simpleIMU.ino
  @brief      Displays data from all 3 sensors of the MPU-6500,
              Gyro, Accelerometer and chip temperature.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        20/03/23

  1.0.0     Original Release.       20/03/23

******************************************************************/

#include <ReefwingMPU6x00.h>

static const uint8_t MOSI_PIN = 8;
static const uint8_t MISO_PIN = 10;
static const uint8_t SCLK_PIN = 9;
static const uint8_t CS_PIN  = 7;
static const uint8_t INT_PIN = 1;
static const uint8_t LED0_PIN = A4;
static const uint8_t LED1_PIN = A5;

float mpuTemp = 0.0f;
int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

static MPU6500 imu = MPU6500(SPI, CS_PIN);
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
    if (gotInterrupt) {
        imu.readSensor();
        imu.getTemp(mpuTemp);
        gotInterrupt = false;
    }

    if (millis() - previousMillis >= displayPeriod) {
    //  Display sensor data every displayPeriod, non-blocking.
    Serial.print("Gyro X: ");
    Serial.print(imu.getGyroX());
    Serial.print("\tGyro Y: ");
    Serial.print(imu.getGyroY());
    Serial.print("\tGyro Z: ");
    Serial.print(imu.getGyroZ());
    Serial.print(" DPS");
  
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

    Serial.print("Accel X: ");
    Serial.print(imu.getAccelX());
    Serial.print("\tAccel Y: ");
    Serial.print(imu.getAccelY());
    Serial.print("\tAccel Z: ");
    Serial.print(imu.getAccelZ());
    Serial.println(" G'S");

    Serial.print("Temp: ");
    Serial.print(mpuTemp);
    Serial.println(" C\n");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}

