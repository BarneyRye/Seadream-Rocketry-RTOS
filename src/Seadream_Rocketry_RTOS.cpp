//If working in arduino IDE, to allow use of esp32 boards
//https://espressif.github.io/arduino-esp32/package_esp32_index.json
//File->Preferences->Additional Board Manager URLs
//Boards Manager -> esp32 by espressif Systems
//Board->ESP32S3 DEV module


//Libaries used and need installing:
/*
Adafruit_LSM6DS
Adafruit SHT4x and dependencies
ESP32 AudioI2S
Adafruit BMP280
*/


//Includes for added libaries
#include <Arduino.h> //Base arduino libary if not using arduino IDE
#include <Wire.h> //I2C protocols
#include <Adafruit_Sensor.h> //Base sensor libary for Adafruit libaries
#include <Adafruit_SHT4x.h> //Temperature and Humidity sensor libaries
#include <Adafruit_LSM6DSO32.h>//Accelerometer and Gyro libary
#include <Adafruit_BMP280.h>//Barometer libary
#include <SD.h> //SD card libary
#include <SPI.h> //SPI libary
#include <Audio.h> //Audio libary
#include <FreeRTOS.h> //FreeRTOS base libary to allow for multitasking
#include <queue.h> //FreeRTOS queue libary for inter-task communication

//Config pins
#define I2C_SDA 12 //I2C pins
#define I2C_SCL 13
#define Audio_DIN 14  //Audio AMP pins
#define Audio_BCLK 15
#define Audio_LRC 16
#define SPI_SCK 2 //SPI/SD card pins
#define SPI_MOSI 3
#define SPI_MISO 1
#define SPI_CS 4

//Log Rates
#define initial_log_rate 50    // Hz, initial logging rate
#define final_log_rate 10      // Hz, reduced logging rate for decent/slower speeds
#define reduce_Hz_time 60      // s, time after which to reduce logging rate, e.g. time to apogee
#define MAX_BUFFER_LINES 5     // No. of data logs per buffer
#define MAX_BUFFER_LENGTH 150  // Length of char buffer lines

//Song timer
#define song_start_time 240  //s, time to start song after startup
#define SONG_PATH "/song.mp3" //Name of song file on SD card

//Object creating from libaries
Adafruit_SHT4x sht4 = Adafruit_SHT4x(); //Temperature and Humidity sensor
Adafruit_LSM6DSO32 imu; //Accelerometer and Gyro
Adafruit_BMP280 bmp; //Barometer
Audio audio; //Audio player

//Data Struct
typedef struct { //struct to store sensor data
  uint32_t time; //TIme of log in ms
  float humidity; //Humidity in %
  float temp_sens; //Temp in C from SHT4x
  float temp_bmp; //Temp in C from BMP180
  float pressure; //Pressure in kPa from BMP180
  float altitude; //Altitude in M from BMP180
  float ax; //Acceleration X in m/s^2
  float ay; //Acceleration Y in m/s^2
  float az; //Acceleration Z in m/s^2
  float gx; //Gyro X in deg/s
  float gy; //Gyro Y in deg/s
  float gz; //Gyro Z in deg/s
} data_struct;

//FreeRTOS Queue
QueueHandle_t logQueue;

//SD Log Filename
char logFilename[15];

//Double Buffers for data logging
data_struct buffer1[MAX_BUFFER_LINES]; // Array of structs to hold data buffers
data_struct buffer2[MAX_BUFFER_LINES];
bool useBuffer1 = true;   // Active buffer flag
uint8_t bufferIndex = 0;  // Index for current buffer

//Fuunction Prototypes
void getNextLogFilename(char* name); //Generate next log filename if log1.csv exists, make log2.csv etc
void getData(data_struct *data_buffer); //Read sensors and fill data struct
void getBufferLine(char *line, data_struct data_buffer); //Converts data struct to string line for block writing to SD card
void char_buffer_write(uint8_t count, data_struct data_buffer, char data_char_buffer[][MAX_BUFFER_LENGTH]); //Writes line to array of char buffers
void audio_start(uint32_t start_time, bool *audio_on); //Checks if it's time to start audio playback
void logRate_reduce(uint32_t *log_rate, bool *reduced, uint32_t loop_start, bool takeoff); //Checks if it's time to reduce log rate and reduces it
void takeoff_detection(bool *takeoff, float initial_altitude); //Detects takeoff based on accelerometer data

//FreeRTOS Tasks
void SensorAudioTask(void *pvParameters); //Core 1 to handle sensor reading and audio playback
void LoggingTask(void *pvParameters); //Core 0 to handle logging data to SD card

//Setup
void setup() {
  Serial.begin(115200); //Begin serial communication
  while(!Serial){delay(10);} //Wait for serial to connect
  Serial.println(F("Initialising...")); //Print initialising message

  //I2C
  Wire.begin(I2C_SDA, I2C_SCL); //Connect I2C to specified pins
  Wire.setClock(100000); //Set I2C clock frequency to 100kHz

  //SPI
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); //Begin SPI for SD card

  //Accelerometer and Gyro
  if (!imu.begin_I2C()) { Serial.println(F("Failed to find Accelerometer")); } //Begins I2C connection to Accel/Gyro, if fails, print failure to serial
  imu.setAccelDataRate(LSM6DS_RATE_52_HZ); //Sets accelerometer data rate to 52Hz
  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G); //Sets accelerometer range to 32G
  imu.setGyroDataRate(LSM6DS_RATE_52_HZ); //Sets gyro data rate to 52Hz
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS); //Sets gyro range to 2000 degrees per second

  //Barometer
  if (!bmp.begin(0x76)) { Serial.println(F("Failed to find Barometer")); } //Begins I2C connection to Barometer, if fails, print failure to serial
  else{
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X1,
                    Adafruit_BMP280::SAMPLING_X1,    
                    Adafruit_BMP280::FILTER_OFF,  
                    Adafruit_BMP280::STANDBY_MS_1); 
  }
  //Temperature and Humidity Sensor
  if (!sht4.begin(&Wire)) { Serial.println(F("Temperature/Humidity sensor not found")); } //Begins I2C connection to Temp/Humidity sensor, if fails, print failure to serial

  //SD CARD
  if (!SD.begin(SPI_CS)) { //Begins SD card connection, if fails, print failure to serial
    Serial.println(F("SD Card not found"));
  } 
  else {
    getNextLogFilename(logFilename); //Get next available log filename
    Serial.print(F("Log file: ")); Serial.println(logFilename); //Prints log filename to serial
    File f = SD.open(logFilename, FILE_WRITE); //Opens/creates file for writing
    if (f) { //If file opened successfully
      //Write CSV header line
      f.println("Time_ms,Humidity_%,Temp_C,Temp_BMP_C,Pressure_kPa,Altitude_M,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z");
      f.close(); //Closes file
    } else {
      Serial.println(F("Failed to open log file")); //Prints failure message to serial
    }
  }

  //Audio
  audio.setPinout(Audio_BCLK, Audio_LRC, Audio_DIN); //Connects audio AMP to specified pins
  audio.setVolume(10);  // 1-21, Sets volume

  //FreeRTOS Queue
  logQueue = xQueueCreate(2, sizeof(bool)); //Create queue to hold 2 buffer ready flags

  //Create FreeRTOS Tasks
  xTaskCreatePinnedToCore(SensorAudioTask, "SensorAudioTask", 4096, NULL, 1, NULL, 1);  // Core 1 for sensors and audio
  xTaskCreatePinnedToCore(LoggingTask, "LoggingTask", 4096, NULL, 1, NULL, 0);          // Core 0 for logging

  Serial.println(F("Finished Initialising")); //Print finished initialising message
}

//Loop
void loop() {
  //Nothing here, all handled by FreeRTOS tasks
}
//Sensor and Audio Task
void SensorAudioTask(void *pvParameters) {
  static bool audio_on = false; //Audio playback flag
  static bool takeoff = false; //Takeoff detected flag
  static uint32_t log_rate = 1000 / initial_log_rate; //Sets initial log rate in ms, converts from Hz
  static bool reduced = false; //Log rate reduced flag
  static uint32_t last_log = 0; //Last log time
  static float initial_altitude = 0.0f; //Initial altitude at startup, used for takeoff detection, value gotten after first data colelction
  static bool first_run = false; //Flag to indicate first run of data collection got intial altitude
  static uint32_t loop_start = millis(); //Loop start time

  for (;;) { //Infinite loop for task
    //Audio Playback
    if (!audio_on) {audio_start(loop_start, &audio_on);} //If audio isn't on, check if it's time to start and start if so
    if (audio_on) { //If audio is on, call audio loop to keep playing
      audio.loop();
      if (!audio.isRunning()) { //If audio has finished playing
        audio.connecttoFS(SD, SONG_PATH); //Restart song
      }
    }

    //Log Rate Reduction
    if (!reduced) logRate_reduce(&log_rate, &reduced, loop_start, takeoff); //If log rate hasn't been reduced yet, check if it's time to reduce it and reduce if so
     
    //Data Logging
    if (millis() - last_log >= log_rate) { //If it's time to log data, i.e. time since last log >= log rate
      uint32_t now = millis(); //Time for current logging iteration
      static uint32_t data_start = now; //First data log time
      
      //Read Sensors and fill data struct
      data_struct data; //Create data struct for current iteration
      data.time = now - data_start; //Stores time since start in data struct
      getData(&data); //Fills data struct with sensor data using function

      last_log = now; //Sets last log time to now

      //Print data to serial for debugging
      Serial.print("Time: "); Serial.print(data.time);
      Serial.print(" ms, Humidity: "); Serial.print(data.humidity); Serial.print(" %, Temp: "); Serial.print(data.temp_sens); Serial.print(" C");
      Serial.print(", Temp_BMP: "); Serial.print(data.temp_bmp); Serial.print(" C, Pressure: "); Serial.print(data.pressure); Serial.print(" kPa, Altitude: "); Serial.print(data.altitude); Serial.print(" m");
      Serial.print(", Accel (m/s^2): X "); Serial.print(data.ax); Serial.print(" Y "); Serial.print(data.ay); Serial.print(" Z "); Serial.print(data.az);
      Serial.print(", Gyro (deg/s): X "); Serial.print(data.gx); Serial.print(" Y "); Serial.print(data.gy); Serial.print(" Z "); Serial.println(data.gz); 

      //Fill active buffer
      if (useBuffer1) { //If flag is true, fill buffer1
        buffer1[bufferIndex] = data; //Copy data struct to buffer1 at current index
      } else { //Else fill buffer2
        buffer2[bufferIndex] = data; //Copy data struct to buffer2 at current index
      }
      bufferIndex++; //Increases buffer index

      //If buffer full, send to logging task
      if (bufferIndex >= MAX_BUFFER_LINES) {
        bool readyBuffer = useBuffer1;  // Sets flag for which buffer is ready
        xQueueSend(logQueue, &readyBuffer, portMAX_DELAY); //Sends ready buffer flag to logging task
        bufferIndex = 0; //Resets buffer index
        useBuffer1 = !useBuffer1;  //Swap active buffer to fill
      }
      initial_altitude = data.altitude; //Initial altitude at startup
      first_run = true;
    }

    if (!takeoff && first_run) { //Calls takeoff detection function to update takeoff flag
      takeoff_detection(&takeoff, initial_altitude);
      loop_start = millis(); //Reset loop start time untill takeoff detected
    }
    vTaskDelay(1);  //Yield to other tasks
  }
}

//Logging Task
void LoggingTask(void *pvParameters) {
  char data_char_buffer[MAX_BUFFER_LINES][MAX_BUFFER_LENGTH]; //Char buffer array for block writing to SD card

  for (;;) { //Infinite loop for task
    bool readyBuffer; //Variable to hold which buffer is ready
    if (xQueueReceive(logQueue, &readyBuffer, portMAX_DELAY) == pdPASS) { //Waits for ready buffer flag from sensor task
      data_struct *buffer = readyBuffer ? buffer1 : buffer2; //if readyBuffer is true, use buffer1, else use buffer2

      //Convert data structs to char buffer lines
      for (uint8_t i = 0; i < MAX_BUFFER_LINES; i++) { //Loops between buffer lines
        char_buffer_write(i, buffer[i], data_char_buffer); //Converts struct data lines to char buffer lines and inserts into corressponding index
      }

      //Write char buffer lines to SD card
      File f = SD.open(logFilename, FILE_APPEND); //Opens log file for appending
      if (f) { //If file opened successfully
        for (uint8_t i = 0; i < MAX_BUFFER_LINES; i++) { //Loops between buffer lines
          f.println(data_char_buffer[i]); //Writes char buffer line to file
        }
        f.close(); //Closes file
      }
    }
  }
}

//Helper Functions
void getNextLogFilename(char *name) { //Gets next available log filename
  uint8_t index = 1; //Starts at log1.csv
  char temp_name[15]; //Temporary filename storage
  while (true) { //Infinite loop until available filename found, doesn't slow logging as done before looping starts
    sprintf(temp_name,"/log%d.csv",index); //Creates filename string from index
    if (!SD.exists(temp_name)){ //If file doesn't exist, copy to output and return
      strcpy(name, temp_name); //Copy temporary name to output
      return; //Exit function
    }
    index++; //Increase index and try again
  }
}

void audio_start(uint32_t start_time, bool *audio_on) { //Checks if it's time to start audio playback
  if (millis() - start_time >= (song_start_time * 1000)) { //If time since start >= song start time
    audio.connecttoFS(SD, SONG_PATH); //Connect to song file on SD card and start playing
    *audio_on = true; //Set audio on flag to true
  }
}

void takeoff_detection(bool *takeoff, float initial_altitude) { //Detects takeoff based on accelerometer data
  if (useBuffer1){
    if (buffer1[bufferIndex].altitude - initial_altitude > 10){ //If altitude has increased by more than 10m since start
      *takeoff = true; //Sets takeoff flag to true
    }
  }
  else{
    if (buffer2[bufferIndex].altitude - initial_altitude > 10){ //If altitude has increased by more than 10m since start
      *takeoff = true; //Sets takeoff flag to true
    }
  }
}

void logRate_reduce(uint32_t *log_rate, bool *reduced, uint32_t loop_start, bool takeoff) { //Checks if it's time to reduce log rate and reduces it
  if ((millis() - loop_start >= (reduce_Hz_time * 1000)) && takeoff) { //If time since start >= reduce time
    *log_rate = 1000 / final_log_rate; //Sets log rate to final log rate in ms, converts from Hz
    *reduced = true; //Sets reduced flag to true
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                    Adafruit_BMP280::SAMPLING_X2,   
                    Adafruit_BMP280::SAMPLING_X16, 
                    Adafruit_BMP280::FILTER_X16,     
                    Adafruit_BMP280::STANDBY_MS_63);
  }
  imu.setAccelDataRate(LSM6DS_RATE_12_5_HZ); //Sets accelerometer data rate to 12.5Hz
  imu.setGyroDataRate(LSM6DS_RATE_12_5_HZ); //Sets gyro data rate to 12.5Hz
}

void getData(data_struct *data_buffer) { //Reads sensors and fills data struct
  //Accelerometer and Gyro
  sensors_event_t accel, gyro, temp_imu; //Create sensor event structs
  imu.getEvent(&accel, &gyro, &temp_imu); //Reads sensor data into structs
  data_buffer->ax = accel.acceleration.x; //Fills data struct with accel and gyro data
  data_buffer->ay = accel.acceleration.y;
  data_buffer->az = accel.acceleration.z;
  data_buffer->gx = gyro.gyro.x;
  data_buffer->gy = gyro.gyro.y;
  data_buffer->gz = gyro.gyro.z;

  //Barometer
  data_buffer->temp_bmp = bmp.readTemperature(); //Reads data from BMP280 and fills data struct
  data_buffer->pressure = bmp.readPressure() / 1000; //Convert Pa to kPa
  data_buffer->altitude = bmp.readAltitude(1013.25); //Assumes sea level pressure of 101325 Pa

  //Temperature and Humidity Sensor
  sensors_event_t humidity, temp; //Create sensor event structs
  sht4.getEvent(&humidity, &temp); //Reads sensor data into structs
  data_buffer->humidity = humidity.relative_humidity; //Fills data struct with temp and humidity data
  data_buffer->temp_sens = temp.temperature;
}

void getBufferLine(char *line, data_struct data_buffer) { //Converts data struct to string line for block writing to SD card
  snprintf(line, MAX_BUFFER_LENGTH, //line is where the string is stored, MAX_BUFFER_LENGTH is max length of line
    "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", //Format string for CSV line, with 2 decimal places for floats except 3 for accel/gyro
    data_buffer.time, //Corressponding data struct values to store in line
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

void char_buffer_write(uint8_t count, data_struct data_buffer, char data_char_buffer[][MAX_BUFFER_LENGTH]) { //Writes line to array of char buffers
  char line[MAX_BUFFER_LENGTH]; //Temporary line buffer
  getBufferLine(line, data_buffer); //Gets data line from data struct
  strcpy(data_char_buffer[count], line); //Copies line to char buffer array at specified index
}
