#include <Arduino.h>

/* Nano Flight Recorder


Hardware setup:
MPU9250 Breakout --------- Arduino
VDD ---------------------- 3.3V
VDDI --------------------- 3.3V
SDA ----------------------- A4
SCL ----------------------- A5
GND ---------------------- GND
*/

#include "i2c_BMP280.h"
#include <SD.h>

#define DEBUG 1  // level 0, 1, or 2

#define debugPrint(level, arg) if(DEBUG >= level) Serial.print(arg);
#define debugPrintln(level, arg) if(DEBUG >= level) Serial.println(arg);

BMP280 bmp280;

#define BAROM_COUNT_THROTTLE 32
#define IMU_COUNT_THROTTLE 32
#define BAROM_TIME_THROTTLE 200
#define IMU_TIME_THROTTLE 200

#define SOFTWARE_VERSION 2

#define BUFFER_SIZE 128

#define MEAS_TYPE_START 0
#define MEAS_TYPE_AHRS 1
#define MEAS_TYPE_ACCELERATION 2
#define MEAS_TYPE_GYRO 3
#define MEAS_TYPE_MAG 4
#define MEAS_TYPE_BAROM 5

struct DataStart {
  uint8_t version; //add whatever needed data, after the version
  float MPU9250testResult[6]; // data provided by MPU9250SelfTest
  float magCalibration[3]; // data provided by initAK8963
  float gyroBias[3]; // data provided by calibrateMPU9250
  float accelBias[3]; // data provided by calibrateMPU9250
  float aRes; // data provided by getAres
  float gRes; // data provided by getGres
  float mRes; // data provided by getMres
} __attribute__((packed));

struct DataAhrs {
  float yaw, pitch, roll;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
} __attribute__((packed));

struct DataAccel {
  int16_t ax, ay, az;
} __attribute__((packed));

struct DataGyro {
  int16_t gx, gy, gz;
} __attribute__((packed));

struct DataMag {
  int16_t mx, my, mz;
} __attribute__((packed));

struct DataBarom {
  uint32_t pressure; // Pascal
  int32_t temperature; // miliC
} __attribute__((packed));

union DataUnion {
  DataStart start;
  DataAhrs ahrs;
  DataAccel acceleration;
  DataGyro gyro;
  DataMag mag;
  DataBarom barom;
} __attribute__((packed));

/** Header structure:
*  4-bit type
*  28-bit timestamp
*/
struct Measurement {
  uint32_t header;
  DataUnion data;
} __attribute__((packed));

uint8_t dataSizeLUT[16] =
{
  sizeof(DataStart),
  sizeof(DataAhrs),
  sizeof(DataAccel),
  sizeof(DataGyro),
  sizeof(DataMag),
  sizeof(DataBarom),
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};


uint32_t baromTriggerTime;
uint32_t imuTriggerTime;

uint8_t buffer[BUFFER_SIZE];
uint16_t bufPos = 0;

#define SD_CS 6
#define LED_PIN 9

// make it long enough to hold your longest file name, plus a null terminator
char filename[16] = "DAT";
File dataFile;

Measurement currentMeasurement;
//manually input desired filename here before each test
//char *s = "ROCK";

void sdSetup()
{
  debugPrint(1, "SD:");

  if (!SD.begin(SD_CS)) {
    debugPrintln(1, "Card fail");
    return;
  }
  debugPrintln(1, "ok.");
  //snprintf(filename, sizeof(filename), "%s%03d.bin", s, n); // includes a two-digit sequence number in the file name
  filename[4] = '0';
  filename[5] = 0;
  filename[3] = '0';
  while (SD.exists(filename)) {
    if(filename[4] == '9')
    {
      filename[3]++;
      filename[4] = '0';
    }
    else
      filename[4]++;
    //snprintf(filename, sizeof(filename), "%s%03d.bin", s, n);
  }
  dataFile = SD.open(filename, FILE_READ);
  debugPrint(1, "file: ");
  debugPrintln(1, filename);
  dataFile.close();
  //now filename[] contains the name of a file that doesn't exist

  pinMode(LED_PIN, OUTPUT);
  dataFile = SD.open(filename, FILE_WRITE);
}

void dataWrite(uint8_t* data, uint16_t size)
{
  debugPrint(2, "write: ");
  debugPrintln(2, size);
  if (dataFile) {
    static boolean ledState = LOW;             // ledState used to set the LED
    unsigned long currentMillis = millis();
    static unsigned long previousMillis = 0;
    if (currentMillis - previousMillis >= 500) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      ledState = !ledState;
      //digitalWrite(LED_PIN, ledState);
    }
    dataFile.write(data, size);
    dataFile.flush();
  }
  else {
    debugPrint(1, "write error");
  }
}

void flushBuffer()
{
  dataWrite(buffer, bufPos);
  bufPos = 0;
}
void handleBarom()
{
  baromTriggerTime = micros();
  bmp280.triggerMeasurement();
  bmp280.awaitMeasurement();
  currentMeasurement.header = ((uint32_t)MEAS_TYPE_BAROM << 28) | (baromTriggerTime & 0x0FFFFFFF);
  bmp280.getTemperature(currentMeasurement.data.barom.temperature);
  bmp280.getPressure(currentMeasurement.data.barom.pressure);
  storeMeasurement(currentMeasurement);
#if (DEBUG >= 1)
  static float meters, metersold;
  bmp280.getAltitude(meters);
  metersold = (metersold * 10 + meters)/11;
  Serial.print("h_old: ");
  Serial.print(metersold);
  Serial.print(" m; h: ");
  Serial.print(meters);
  Serial.print(" P: ");
  Serial.print(currentMeasurement.data.barom.pressure);
  Serial.print(" Pa; T: ");
  Serial.print(currentMeasurement.data.barom.temperature);
  Serial.println(" C");
#endif
}

void storeMeasurement(Measurement& meas)
{
  uint8_t measType = meas.header >> 28;
  uint8_t measSize = dataSizeLUT[measType] + 4; // header has 4 bytes
#if (DEBUG >= 2)
  Serial.print("Storing: ");
  Serial.print(measType);
  Serial.print(" size: ");
  Serial.println(measSize);
#endif
  if (bufPos + measSize > BUFFER_SIZE)
    flushBuffer();
  memcpy((void*)(buffer + bufPos), (void*)&meas, measSize);
  bufPos += measSize;
}

void setup()
{
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  sdSetup();


  #if (DEBUG >= 1)
  Serial.print("Probe BMP: ");
  if (bmp280.initialize())
  {
    Serial.println("found");
  }
  else
  {
    Serial.println("missing");
  }
  #else
  bmp280.initialize();
  #endif

  // onetime-measure:
  bmp280.setEnabled(0);
}

uint16_t baromCounter = 0;
uint16_t baromLastTime = 0;
void loop()
{
  //if (baromCounter >= BAROM_COUNT_THROTTLE && millis()-baromLastTime >= BAROM_TIME_THROTTLE)
  //{
    baromCounter = 0;
    baromLastTime = millis();
    handleBarom();
  //}
  //handleIMU();
  baromCounter++;
}
