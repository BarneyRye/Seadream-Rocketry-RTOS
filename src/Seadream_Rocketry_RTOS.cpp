//https://espressif.github.io/arduino-esp32/package_esp32_index.json
//File->Preferences->Additional Board Manager URLs
//Boards Manager -> esp32 by espressif Systems
//Board->ESP32S3 DEV module

//Libraries includes:
//Adafruit_LSM6DS
//Adafruit SHT4x and dependencies
//ESP32 AudioI2S
//Adafruit BMP085

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT4x.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Audio.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_BMP085.h>
#include <FreeRTOS.h>
#include <queue.h>

//I2C config
#define I2C_SDA 8
#define I2C_SCL 9
#define Audio_DIN 25
#define Audio_BCLK 26
#define Audio_LRC 27
#define SD_CS 6

//Log Rates
#define initial_log_rate 50    // Hz
#define final_log_rate 10      // Hz
#define reduce_Hz_time 60      // s
#define MAX_BUFFER_LINES 5     // lines per buffer
#define MAX_BUFFER_LENGTH 150  // char length per line

//Song timer
#define song_start_time 240  // s
#define SONG_PATH "/song.mp3"

//SENSOR AND AUDIO OBJECTS
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_LSM6DSO32 imu;
Adafruit_BMP085 bmp;
Audio audio;

//DATA STRUCTURE
typedef struct {
  uint32_t time;
  float humidity;
  float temp_sens;
  float temp_bmp;
  float pressure;
  float altitude;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} data_struct;

//FREERTOS QUEUE
QueueHandle_t logQueue;

//SD LOGGING
char logFilename[15];

//DOUBLE BUFFERS
data_struct buffer1[MAX_BUFFER_LINES];
data_struct buffer2[MAX_BUFFER_LINES];
bool useBuffer1 = true;   // active buffer flag
uint8_t bufferIndex = 0;  // index for current buffer

//FUNCTION PROTOTYPES
void getNextLogFilename(char* name);
void getData(data_struct *data_buffer);
void getBufferLine(char *line, data_struct data_buffer);
void char_buffer_write(uint8_t count, data_struct data_buffer, char data_char_buffer[][MAX_BUFFER_LENGTH]);
void audio_start(uint32_t start_time, bool *audio_on);
void logRate_reduce(uint32_t *log_rate, bool *reduced, uint32_t loop_start);

//FREERTOS TASK PROTOTYPES
void SensorAudioTask(void *pvParameters);
void LoggingTask(void *pvParameters);

//SETUP
void setup() {
  Serial.begin(115200);
  Serial.println(F("Initialising..."));

  //I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  //SPI
  SPI.begin();

  //ACCELEROMETER AND GYRO
  if (!imu.begin_I2C()) { Serial.println(F("Failed to find Accelerometer")); }
  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  //BAROMETER
  if (!bmp.begin()) { Serial.println(F("Failed to find Barometer")); }

  //TEMPERATURE AND HUMIDITY
  if (!sht4.begin(&Wire)) { Serial.println(F("Temperature/Humidity sensor not found")); }

  //SD CARD
  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD Card not found"));
  } else {
    getNextLogFilename(logFilename);
    File f = SD.open(logFilename, FILE_WRITE);
    if (f) {
      f.println("Time_ms,Humidity_%,Temp_C,Temp_BMP_C,Pressure_kPa,Altitude_M,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z");
      f.close();
    } else {
      Serial.println(F("Failed to open log file"));
    }
  }

  //AUDIO
  audio.setPinout(Audio_BCLK, Audio_LRC, Audio_DIN);
  audio.setVolume(15);  // 1-21

  //FREERTOS QUEUE
  logQueue = xQueueCreate(2, sizeof(bool));

  //CREATE TASKS
  xTaskCreatePinnedToCore(SensorAudioTask, "SensorAudioTask", 4096, NULL, 1, NULL, 1);  // Core 1
  xTaskCreatePinnedToCore(LoggingTask, "LoggingTask", 4096, NULL, 1, NULL, 0);          // Core 0

  Serial.println(F("Finished Initialising"));
}

//LOOP
void loop() {
  //Nothing here, all handled by FreeRTOS tasks
}

//SENSOR AND AUDIO TASK
void SensorAudioTask(void *pvParameters) {
  static bool audio_on = false;
  static uint32_t loop_start = millis();
  static uint32_t log_rate = 1000 / initial_log_rate;
  static bool reduced = false;
  static uint32_t last_log =0;

  for (;;) {
    uint32_t now = millis();

    //AUDIO HANDLING
    if (!audio_on) {audio_start(loop_start, &audio_on);}
    if (audio_on) {
      audio.loop();
      if (!audio.isRunning()) {
        audio.connecttoFS(SD, SONG_PATH);
      }
    }

    //LOG RATE REDUCTION
    if (!reduced) logRate_reduce(&log_rate, &reduced, loop_start);

    //SENSOR LOGGING
    if (now - last_log >= log_rate) {
      static uint32_t data_start = millis();
      last_log = now;

      //READ SENSORS
      data_struct data;
      data.time = now - data_start;
      getData(&data);

      //FILL ACTIVE BUFFER
      if (useBuffer1) {
        buffer1[bufferIndex] = data;
      } else {
        buffer2[bufferIndex] = data;
      }
      bufferIndex++;

      //IF BUFFER FULL, SEND TO LOGGING TASK AND SWAP
      if (bufferIndex >= MAX_BUFFER_LINES) {
        bool readyBuffer = useBuffer1;  // true = buffer1 ready, false = buffer2
        xQueueSend(logQueue, &readyBuffer, portMAX_DELAY);
        bufferIndex = 0;
        useBuffer1 = !useBuffer1;  // swap active buffer
      }
    }

    vTaskDelay(1);  //Yield to other tasks
  }
}

//LOGGING TASK
void LoggingTask(void *pvParameters) {
  char data_char_buffer[MAX_BUFFER_LINES][MAX_BUFFER_LENGTH];

  for (;;) {
    bool readyBuffer;
    if (xQueueReceive(logQueue, &readyBuffer, portMAX_DELAY) == pdPASS) {
      data_struct *buffer = readyBuffer ? buffer1 : buffer2;

      //CONVERT BUFFER DATA TO STRING LINES
      for (uint8_t i = 0; i < MAX_BUFFER_LINES; i++) {
        char_buffer_write(i, buffer[i], data_char_buffer);
      }

      //WRITE BUFFER TO SD CARD
      File f = SD.open(logFilename, FILE_APPEND);
      if (f) {
        for (uint8_t i = 0; i < MAX_BUFFER_LINES; i++) {
          f.println(data_char_buffer[i]);
        }
        f.close();
      }
    }
  }
}

//HELPER FUNCTIONS
void getNextLogFilename(char *name) {
  uint8_t index = 1;
  char temp_name[15];
  while (true) {
    sprintf(temp_name,"/log%d.csv",index);
    if (!SD.exists(temp_name)){
      strcpy(name, temp_name);
      return;
    }
    index++;
  }
}

void audio_start(uint32_t start_time, bool *audio_on) {
  if (millis() - start_time >= (song_start_time * 1000)) {
    audio.connecttoFS(SD, SONG_PATH);
    *audio_on = true;
  }
}

void logRate_reduce(uint32_t *log_rate, bool *reduced, uint32_t loop_start) {
  if (millis() - loop_start >= (reduce_Hz_time * 1000)) {
    *log_rate = 1000 / final_log_rate;
    *reduced = true;
  }
}

void getData(data_struct *data_buffer) {
  //ACCELEROMETER
  sensors_event_t accel, gyro, temp_imu;
  imu.getEvent(&accel, &gyro, &temp_imu);
  data_buffer->ax = accel.acceleration.x;
  data_buffer->ay = accel.acceleration.y;
  data_buffer->az = accel.acceleration.z;
  data_buffer->gx = gyro.gyro.x;
  data_buffer->gy = gyro.gyro.y;
  data_buffer->gz = gyro.gyro.z;

  //BAROMETER
  data_buffer->temp_bmp = bmp.readTemperature();
  data_buffer->pressure = bmp.readPressure() / 1000;
  data_buffer->altitude = bmp.readAltitude(101325);

  //TEMPERATURE AND HUMIDITY
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);
  data_buffer->humidity = humidity.relative_humidity;
  data_buffer->temp_sens = temp.temperature;
}

void getBufferLine(char *line, data_struct data_buffer) {
  snprintf(line, MAX_BUFFER_LENGTH,
           "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
           data_buffer.time,
           data_buffer.humidity,
           data_buffer.temp_sens,
           data_buffer.temp_bmp,
           data_buffer.pressure,
           data_buffer.altitude,
           data_buffer.ax,
           data_buffer.ay,
           data_buffer.az,
           data_buffer.gx,
           data_buffer.gy,
           data_buffer.gz);
}

void char_buffer_write(uint8_t count, data_struct data_buffer, char data_char_buffer[][MAX_BUFFER_LENGTH]) {
  char line[MAX_BUFFER_LENGTH];
  getBufferLine(line, data_buffer);
  strcpy(data_char_buffer[count], line);
}
