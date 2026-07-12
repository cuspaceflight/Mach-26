#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "SdFat.h"
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <7Semi_ICM20948.h>
#include <RadioLib.h>

//Pin Definitions
const uint8_t SD_MISO = 4;
const uint8_t SD_MOSI = 3;
const uint8_t SD_SCK = 2;
const uint8_t SD_CS = 17;

const uint8_t Radio_MISO = 12;    
const uint8_t Radio_MOSI = 11;
const uint8_t Radio_SCK = 10;
const uint8_t Radio_NSS = 13;
const uint8_t Radio_DIO1 = 21;
const uint8_t Radio_NRST = 15;
const uint8_t Radio_BUSY = 22;

#define BME_SCK 2
#define BME_MISO 4
#define BME_MOSI 3
#define BME_CS 5

//Configurations
#define GPS_Serial Serial2
#define GPS_FREQ 38400
#define GPS_TIMEOUT 1000
#define ICM_ADDR 0x68
#define Radio_FREQ 868 //In MHz
#define SD_CONFIG SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
#define SEALEVELPRESSURE_HPA (1013.25)
#define Radio_TIMER 10000

//Class Objects
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
TinyGPSPlus gps;
ICM20948_7Semi icm;
SdFs sd;
FsFile file;
SoftSpiDriver<SD_MISO, SD_MOSI, SD_SCK> softSpi;
SX1262 radio = new Module(Radio_NSS, Radio_DIO1, Radio_NRST, Radio_BUSY, SPI1);

//Measured variables
float ax, ay, az;  // accel X/Y/Z in g
float gx, gy, gz; // gyro readings
float icm_temp; //temperature in °C
float bme_temp, humidity, pressure, gas, bme_alt;
float lat, lon, gps_alt, speed;

//helper variables
uint32_t timer1 = 0;
uint32_t timer2 = 0;
uint32_t timer3 = 0;
uint32_t timer4 = 0;
uint8_t result = 0;
uint32_t counter = 0;
bool sdValid = 0;
static uint8_t buffer[24];

typedef struct LoraMessage_T {
    uint32_t timestamp;
    uint32_t entry;
    float gps_lat;
    float gps_lon;
    float gps_alt;
    float gps_speed;
} LoraMessage;

void setup() {
  //Serial.begin(115200);
  //while(!Serial){}
  
  initGPS();
  initSD();
  initBME();
  initICM();
  initRadio();
  
  timer1 = millis();
  timer2 = millis();
}

void loop() {
  readGPS();
  readBME();
  readICM();

  if (sdValid){
    logData();
  }

  timer2 = millis();
  if (timer2 - timer1 > Radio_TIMER){
    counter++;
    form_message(buffer, sizeof(LoraMessage));
    radio.transmit(buffer, sizeof(LoraMessage));
    timer1 = millis();
    //Serial.println("Packet sent!");
  }
  
  String data = ""; 
  data = String(lat) + "," + String(lon) + "," + String(gps_alt) + "," + String(speed) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," + String(icm_temp) + "," + String(bme_temp) + "," + String(humidity) + "," + String(pressure) + "," + String(gas) + "," + String(bme_alt) + "," + String(speed);
  //Serial.println(data);
  //Serial.println(F("-----------------------------"));
  delay(100);  
}

//Initialisation functions
int initBME(){
  if (!bme.begin()) {
    //Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    return 0;
  }
  //Serial.println("BME-680 Initialised!");
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
  return 1;
}

int initICM(){
  Wire1.begin();
  if (!icm.begin(Wire1, ICM_ADDR)) {
    //Serial.println(F("ERROR: ICM-20948 I2C begin() failed."));
    return 0;
  }
  else{
    //Serial.println(F("ICM-20948 initialised!"));
  }
  icm.setSensors(true, true, true); //accel, gyro, temp
  icm.AccelConfigure(ACCEL_DLPFCFG_3, g4, true, ACCEL_DEC3_AVG_8, false, false, false);
  icm.GyroConfig(GYRO_DLPFCFG_4, dps2000, true, true, true, true, 0);
  icm.Accel_SMPLRT(225);
  icm.Gyro_SMPLRT(1000);
  return 1;
}

int initSD(){
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt();
    return 0;
  }
  if (!file.open("data.csv", O_RDWR | O_CREAT | O_APPEND)) {
    sd.errorHalt(F("File open failed"));
    return 0;
  }
  sdValid = 1;
  //Serial.println("SD Card Reader initialised!");
  return 1;
}

int initRadio(){
  SPI1.setRX(Radio_MISO);    // MISO
  SPI1.setTX(Radio_MOSI);    // MOSI
  SPI1.setSCK(Radio_SCK);   // SCK
  SPI1.begin();

  ConfigLoRa_t config;
  config.frequency = Radio_FREQ;
  int state = radio.begin(config);

  if (state == RADIOLIB_ERR_NONE) {
    //Serial.println("Radio Initialised!");
  }
  else {
    //Serial.print("Radio failed with error code ");
    //Serial.println(state);
    return state;
  }
  
  radio.setSyncWord(0x3444);
  return 1;
}

int initGPS(){
  GPS_Serial.begin(GPS_FREQ);
  while(!GPS_Serial);
  //Serial.println("GPS Initialised!");
  return 1;
}

//Measurement functions
void readBME(){
  if (! bme.performReading()) {
    //Serial.println("Failed to perform reading :(");
    return;
  }
  bme_temp = bme.temperature;
  humidity = bme.humidity;
  pressure = bme.pressure;
  gas = bme.gas_resistance;
  bme_alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void readICM(){
  icm.readAccel(ax, ay, az);
  icm.readTemperature(icm_temp);
  icm.readGyro(gx, gy, gz);
}

void logData(){
  String data = "";
  data = String(millis()) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(icm_temp) + "," + String(bme_temp) + "," + String(humidity) + "," + String(pressure) + "," + String(gas) + "," + String(bme_alt) + "," + String(speed);
  file.println(data);
  file.flush();
}

int form_message(uint8_t* buffer, size_t max_size) {
  LoraMessage message;
  message.timestamp = millis();
  message.entry = counter;

  message.gps_lat = lat;
  message.gps_lon = lon;
  message.gps_alt = gps_alt;
  message.gps_speed = speed;

  memcpy(buffer, &message, sizeof(LoraMessage));

  return sizeof(LoraMessage);
}

void readGPS(){
  if (GPS_Serial.available()) {
    bool updated = false;
    timer3 = millis();
    timer4 = millis();
    while (!updated && timer4 - timer3 < GPS_TIMEOUT){
      timer4 = millis();
      while (GPS_Serial.available()) {
        char c = GPS_Serial.read();
        gps.encode(c);
        updated = gps.location.isUpdated();
        if (updated){
          lat = gps.location.lat();
          lon = gps.location.lng();
          gps_alt = gps.altitude.meters();
          speed = gps.speed.kmph();

          GPS_Serial.flush();
          break;
        }
      }
    }
    updated = false;
  }
}