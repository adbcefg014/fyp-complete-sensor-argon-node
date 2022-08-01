// Code for all one complete sensor node
#include <Particle.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>	
#include <Adafruit_BME280.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_VEML6070.h>
#include <sps30.h>

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

BH1750 bh;
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_ADDRESS 0x77
Adafruit_BME280 bme;
SCD30 airSensor;
#define SP30_COMMS I2C_COMMS
#define TX_PIN 0
#define RX_PIN 0
#define DEBUG 0
SPS30 sps30;
Adafruit_VEML6070 uv = Adafruit_VEML6070();
#define COMMAND_GET_VALUE 0x05
#define COMMAND_NOTHING_NEW 0x99
const byte qwiicAddress = 0x30;
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
uint16_t ADC_VALUE = 0;
float dBnumber = 0.0;
unsigned long sensingInterval = 60000;
time_t timeNow;
SystemSleepConfiguration sleepConfig;
bool waitedForSPS30 = false;
int readingsToCollate = 1;

void initializeSensors();
JSONBufferWriter getSensorReadings(JSONBufferWriter writerData);
void qwiicTestForConnectivity();
void qwiicGetValue();
JSONBufferWriter readSPS30(JSONBufferWriter writerData);
void finishWaitForSPS30();
Timer delayForSPS30 (30000, finishWaitForSPS30, true);
void goSleep();
void syncClock();

// setup() runs once, when the device is first turned on.
void setup() {
	Particle.connect();
	pinMode(D7,OUTPUT);
	Wire.begin();
	Serial.begin();
	initializeSensors();

	// Cloud sync initialization
	waitUntil(Particle.connected);
	Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));
	Particle.publishVitals();
	Particle.process();
	syncClock();

	// Wait for background tasks to finish, and SCD30 to start up
	delay(60s);

	// Turn off connectivity
	Particle.disconnect();
	waitUntil(Particle.disconnected);
	WiFi.off();

	// Start first sensor reading timer now
	timeNow = Time.now();
}

// loop() runs over and over again, as quickly as it can execute. Main Program flow goes here!
void loop() {
	// Start of sensor data string
	char *dataString = (char *) malloc(1050);
	JSONBufferWriter writerData(dataString, 1049);
	writerData.beginArray();
	writerData.value(System.deviceID());

	// Collate sensor readings sets into 1 string
	for (int collateCount = 0; collateCount < readingsToCollate; collateCount++){

		// Sleep until next sensor reading timing
		sleepConfig.mode(SystemSleepMode::ULTRA_LOW_POWER)
					.duration(sensingInterval + timeNow - Time.now());
		System.sleep(sleepConfig);

		// Start round of reading sensor data
		digitalWrite(D7,HIGH);
		sps30.wakeup();
		sps30.start();
		delayForSPS30.start();
		bh.make_forced_measurement();

		// Preparation for publishing in last loop, wake up various stuff
		if (collateCount == (readingsToCollate - 1)) {
			Serial.begin();
			WiFi.on();
			Particle.connect();
		}

		// Collate readings into 1 JSON string
		String readingName = String::format("r%i", collateCount + 1);
		writerData.beginArray();
			timeNow = Time.now();
			writerData.value(Time.format(timeNow, TIME_FORMAT_ISO8601_FULL));
			writerData = getSensorReadings(writerData);
		writerData.endArray();
		digitalWrite(D7,LOW);
	}

	// End of sensor data string
	writerData.endArray();
	writerData.buffer()[std::min(writerData.bufferSize(), writerData.dataSize())] = 0;
	

	// Publish collated sensor data string
	waitUntil(Particle.connected);
	Serial.println("Collated:");
	Serial.println(writerData.dataSize());
	Serial.println(dataString);
	Serial.println("");
	Particle.publish("sensor-readings", dataString);

	// Sync device clock daily
	Particle.publishVitals();
	Particle.process();
  	syncClock();

	// Shut down connectivity
	Particle.disconnect();
	waitUntil(Particle.disconnected);
	WiFi.off();
	free(dataString);
	digitalWrite(D7,LOW);
	
}

/* Main program flow above */

void initializeSensors()
{
	// BH1750 Lux Sensor
	while (!bh.begin()) {
		delay(500);
		Serial.println("Trying to connect BH1750 Lux Sensor");
	}
	bh.set_sensor_mode(BH1750::forced_mode_low_res);

	// BME280 PTH Sensor, Recommended weather monitoring settings, 1 sample/min
	while (!bme.begin()) {
		delay(500);
		Serial.println("Trying to connect BME280 PTH Sensor");
	}
	bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

	// SCD30 CO2 Sensor, 1 sample/min
	while (!airSensor.begin()) {
		delay(500);
		Serial.println("Trying to connect SCD30 CO2 Sensor");
	}
	airSensor.setMeasurementInterval(55);
  	airSensor.setAutoSelfCalibration(true);

	// Particulate sensor SPS30
	sps30.EnableDebugging(DEBUG);
	while (!sps30.begin(SP30_COMMS)) {
		delay(500);
		Serial.println("Trying to connect Particulate SPS30 Sensor");
	}
	sps30.reset();

	// Zio Qwiic Loudness Sensor Master
	qwiicTestForConnectivity();
	Serial.println("Zio Qwiic Loudness Sensor Master Awake");

	// VEML6070 UV Level Sensor
	uv.begin(VEML6070_1_T);
}

JSONBufferWriter getSensorReadings(JSONBufferWriter writerData)
{
	/*
	* Planned Data Structure:
	* 
	* [
	*	"deviceID",
	* 	["timestamp1", lux, dB, UV, Pressure, Temp, Humidity, CO2, PM1.0, PM2.5, PM4.0, PM10],	//reading set 1
	* 	["timestamp2", lux, dB, UV, Pressure, Temp, Humidity, CO2, PM1.0, PM2.5, PM4.0, PM10],	//reading set 2
	* ]
	*
	*/

	// LUX Sensor (BH1750)
	writerData.value(bh.get_light_level());

	// Peak Sound Sensor (SPARKFUN SEN-15892)
	qwiicGetValue();
	writerData.value(dBnumber);

	// UV Sensor (VEML 6070)
	writerData.value(uv.readUV());

	// Pressure, Temperature, Humidity Sensor (BME280)
	writerData.value(bme.readPressure()/100.0F);
	writerData.value(bme.readTemperature());
	writerData.value(bme.readHumidity());

	// CO2 Sensor (SCD30)
	waitUntil(airSensor.dataAvailable);
	if (airSensor.dataAvailable()) {
		writerData.value(airSensor.getCO2());
	}

	// Particulate Sensor (SPS30)
	while (!waitedForSPS30) delay(1s);
	writerData = readSPS30(writerData);
	sps30.sleep();

	return writerData;
}


void qwiicGetValue()
{
	Wire.beginTransmission(qwiicAddress);
	Wire.write(COMMAND_GET_VALUE); // command for status
	Wire.endTransmission(); // stop transmitting //this looks like it was essential.
	Wire.requestFrom(qwiicAddress, 2); // request 1 bytes from slave device qwiicAddress

	while (Wire.available())
	{ // slave may send less than requested
		uint8_t ADC_VALUE_L = Wire.read();
		uint8_t ADC_VALUE_H = Wire.read();
		ADC_VALUE=ADC_VALUE_H;
		ADC_VALUE<<=8;
		ADC_VALUE|=ADC_VALUE_L;
		dBnumber = (ADC_VALUE+83.2073) / 11.003; //emprical formula to convert ADC value to dB
	}
	return;
}

// qwiicTestForConnectivity() checks for an ACK from an Sensor. If no ACK
// program freezes and notifies user.
void qwiicTestForConnectivity()
{
	Wire.beginTransmission(qwiicAddress);
	//check here for an ACK from the slave, if no ACK don't allow change?
	if (Wire.endTransmission() != 0) {
		Serial.println("Check connections. No slave attached.");
		while (1);
	}
	return;
}

JSONBufferWriter readSPS30(JSONBufferWriter writerData) {
	uint8_t ret;
	struct sps_values val;

	// loop to get data
	do {
		ret = sps30.GetValues(&val);
	} while (ret != SPS30_ERR_OK);
	

	writerData.value(val.MassPM1);
	writerData.value(val.MassPM2);
	writerData.value(val.MassPM4);
	writerData.value(val.MassPM10);
	return writerData;
}

void finishWaitForSPS30() {
	waitedForSPS30 = true;
	return;
}

void syncClock()
{
	unsigned long lastSync = Particle.timeSyncedLast();
	if (millis() - lastSync > ONE_DAY_MILLIS){
		Particle.syncTime();
		waitUntil(Particle.syncTimeDone);
	}
	return;
}