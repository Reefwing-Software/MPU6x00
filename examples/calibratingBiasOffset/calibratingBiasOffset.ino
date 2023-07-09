/******************************************************************
  @file       calibratingBiasOffset.ino
  @brief      Displays data from MPU-6500 after calibration. Keep
              the IMU still and flat during bias offset calibration.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        09/07/23

  1.0.0     Original Release.       09/07/23

******************************************************************/

#include <ReefwingMPU6x00.h>

static const uint8_t MOSI_PIN = 8;
static const uint8_t MISO_PIN = 10;
static const uint8_t SCLK_PIN = 9;
static const uint8_t CS_PIN  = 7;
static const uint8_t INT_PIN = 1;
static const uint8_t LED0_PIN = A4;
static const uint8_t LED1_PIN = A5;

float gx, gy, gz, ax, ay, az, mpuTemp = 0.0f;
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

    //  Calibrate IMU for Bias Offset
    Serial.println("Calibrating IMU - no movement please!");
    imu.calibrateAccelGyro();
    Serial.println("IMU Calibrated.");

    BiasOffsets gyroOffsets = imu.getGyroOffsets();
    BiasOffsets accelOffsets = imu.getAccelOffsets();

    Serial.print("GX Offset: ");
    Serial.print(gyroOffsets.x);
    Serial.print("\tGY Offset: ");
    Serial.print(gyroOffsets.y);
    Serial.print("\tGZ Offset: ");
    Serial.println(gyroOffsets.z);
    Serial.print("AX Offset: ");
    Serial.print(accelOffsets.x);
    Serial.print("\tAY Offset: ");
    Serial.print(accelOffsets.y);
    Serial.print("\tAZ Offset: ");
    Serial.println(accelOffsets.z);
    Serial.println();
}

void loop(void) {
    if (gotInterrupt) {
        imu.readSensor();
        imu.getCalibratedGyro(gx, gy, gz);
        imu.getCalibratedAccel(ax, ay, az);
        imu.getTemp(mpuTemp);
        gotInterrupt = false;
    }

    if (millis() - previousMillis >= displayPeriod) {
    //  Display sensor data every displayPeriod, non-blocking.
    Serial.print("Gyro X: ");
    Serial.print(gx);
    Serial.print("\tGyro Y: ");
    Serial.print(gy);
    Serial.print("\tGyro Z: ");
    Serial.print(gz);
    Serial.print(" DPS");
  
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

    Serial.print("Accel X: ");
    Serial.print(ax);
    Serial.print("\tAccel Y: ");
    Serial.print(ay);
    Serial.print("\tAccel Z: ");
    Serial.print(az);
    Serial.println(" G'S");

    Serial.print("Temp: ");
    Serial.print(mpuTemp);
    Serial.println(" C\n");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}
