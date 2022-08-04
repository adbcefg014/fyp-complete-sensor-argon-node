/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "d:/JSN/Desktop/repos/c177-iot/sensor-node/src/sensor-node.ino"
// Code for all one complete sensor node
#include <Particle.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>	
#include <Adafruit_BME280.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_VEML6070.h>
#include <sps30.h>	// https://github.com/paulvha/sps30 , line 190 edited
#include <math.h>

void setup();
void loop();
#line 13 "d:/JSN/Desktop/repos/c177-iot/sensor-node/src/sensor-node.ino"
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

BH1750 bh;
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_ADDRESS 0x77
Adafruit_BME280 bme;
SCD30 airSensor;
#define SP30_COMMS I2C_COMMS
#define AUTOCLEANINTERVAL 604800
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
unsigned long intervalCompensation = 30000;
unsigned long sensingInterval = 120000;
time_t timeNow;
SystemSleepConfiguration sleepConfig;
#define READINGS_TO_COLLATE 5
int sensorErrorCount = 0;

void initializeSensors();
JSONBufferWriter getSensorReadings(JSONBufferWriter writerData);
void qwiicTestForConnectivity();
void qwiicGetValue();
JSONBufferWriter readSPS30(JSONBufferWriter writerData);
void goSleep();
void syncClock();
void checkErrorReset();
int adjustIntervals (String inString);

// setup() runs once, when the device is first turned on.
void setup() {
	pinMode(D7,OUTPUT);
	Particle.function("adjustIntervals", adjustIntervals);
	Particle.connect();
	Wire.begin();
	Serial.begin();
	sensorErrorCount = 0;
	initializeSensors();

	// Cloud sync initialization
	waitUntil(Particle.connected);
	Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));
	Particle.publishVitals();
	Particle.process();
	syncClock();

	// Wait for background tasks and sensor initialization to finish
	delay(30s);

	// Turn off connectivity
	Particle.disconnect();
	waitUntil(Particle.disconnected);
	WiFi.off();
	sps30.sleep();

	// Start first sensor reading timer now
	timeNow = Time.now();
}

// loop() runs over and over again, as quickly as it can execute. Main Program flow goes here!
void loop() {
	// Start of sensor data string
	char *dataString = (char *) malloc(1050);
	JSONBufferWriter writerData(dataString, 1049);
	writerData.beginArray();

	// Collate sensor readings sets into 1 string
	for (int collateCount = 0; collateCount < READINGS_TO_COLLATE; collateCount++){

		// Sleep until next sensor reading timing
		sleepConfig.mode(SystemSleepMode::ULTRA_LOW_POWER)
					.duration(sensingInterval - intervalCompensation + timeNow - Time.now());
		System.sleep(sleepConfig);

		// Start round of reading sensor data
		digitalWrite(D7,HIGH);
		sps30.wakeup();
		bme.begin();
		bh.begin();

		// Preparation for publishing in last loop, wake up various stuff
		if (collateCount == (READINGS_TO_COLLATE - 1)) {
			Serial.begin();
			WiFi.on();
			Particle.connect();
		}

		// Start 30s measurement time before taking readings
		delay(500);
		sps30.start();
		delay(30s);

		// Collate readings into 1 JSON string
		writerData.beginArray();
		timeNow = Time.now();
		writerData.value(Time.format(timeNow, TIME_FORMAT_ISO8601_FULL));
		writerData = getSensorReadings(writerData);
		writerData.endArray();

		// End sensor reading
		sps30.stop();
		delay(500);
		sps30.sleep();
		digitalWrite(D7,LOW);
	}

	// End of sensor data string
	digitalWrite(D7,HIGH);
	writerData.endArray();
	writerData.buffer()[std::min(writerData.bufferSize(), writerData.dataSize())] = 0;
	Serial.println("taken sensor reading");
	

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
		delay(1s);
		Serial.println("Trying to connect BH1750 Lux Sensor");
		sensorErrorCount++;
		checkErrorReset();
	}
	bh.set_sensor_mode(BH1750::forced_mode_low_res);

	// BME280 PTH Sensor, Recommended weather monitoring settings
	while (!bme.begin()) {
		delay(1s);
		Serial.println("Trying to connect BME280 PTH Sensor");
		sensorErrorCount++;
		checkErrorReset();
	}
	bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

	// SCD30 CO2 Sensor
	while (!airSensor.begin()) {
		delay(1s);
		Serial.println("Trying to connect SCD30 CO2 Sensor");
		sensorErrorCount++;
		checkErrorReset();
	}
	airSensor.setMeasurementInterval(60);
  	airSensor.setAutoSelfCalibration(true);

	// Particulate sensor SPS30
	sps30.EnableDebugging(DEBUG);
	while (!sps30.begin(SP30_COMMS)) {
		delay(1s);
		Serial.println("Trying to connect Particulate SPS30 Sensor");
		sensorErrorCount++;
		checkErrorReset();
	}
	sps30.SetAutoCleanInt(AUTOCLEANINTERVAL);
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
	* 	["timestamp1", lux, dB, UV, Pressure, Temp, Humidity, CO2, PM1.0, PM2.5, PM4.0, PM10],	//reading set 1
	* 	["timestamp2", lux, dB, UV, Pressure, Temp, Humidity, CO2, PM1.0, PM2.5, PM4.0, PM10],	//reading set 2
	* ]
	*
	*/

	// LUX Sensor (BH1750), rounded to 1 decimal place
	bh.make_forced_measurement();					// default 4 decimal place
	float lux = roundf(bh.get_light_level() * 10) / 10;
	writerData.value(lux);

	// Peak Sound Sensor (SPARKFUN SEN-15892), rounded to 1 decimal place
	qwiicGetValue();
	float dBrounded = roundf(dBnumber * 10) / 10;
	writerData.value(dBrounded);

	// UV Sensor (VEML 6070)
	writerData.value(uv.readUV());					// default whole numbers

	// Pressure, Temperature, Humidity Sensor (BME280), rounded to 1 decimal place
	float pressure = roundf((bme.readPressure()/100.0F) * 10) / 10;
	float tempC = roundf(bme.readTemperature() * 10) / 10;
	float humidity = roundf(bme.readHumidity() * 10) / 10;
	writerData.value(pressure);						// default 2 decimal place
	writerData.value(tempC);						// default 2 decimal place
	writerData.value(humidity);						// default 4 decimal place

	// Particulate Sensor (SPS30)
	writerData = readSPS30(writerData);				// default 5 decimal place
	sps30.sleep();

	// CO2 Sensor (SCD30)
	writerData.value(airSensor.getCO2());			// default whole numbers

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

	// pm2 refers to PM2.5 reading
	float pm1 = roundf(val.MassPM1 * 10) / 10;
	float pm2 = roundf(val.MassPM2 * 10) / 10;
	float pm4 = roundf(val.MassPM4 * 10) / 10;
	float pm10 = roundf(val.MassPM10 * 10) / 10;

	writerData.value(pm1);
	writerData.value(pm2);
	writerData.value(pm4);
	writerData.value(pm10);
	return writerData;
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

void checkErrorReset() {
	if (sensorErrorCount == 5) {
		System.reset();
	}
	return;
 }

int adjustIntervals (String inString) {
	// Incoming string should be in the following format:
	// [newSensingInterval, newIntervalCompensation]
	// 
	JSONValue inObj = JSONValue::parseCopy(inString);
	JSONArrayIterator iter(inObj);
	int counter = 0;

	while(iter.next()) {
		if (counter == 0) sensingInterval = (unsigned long)iter.value().toInt();
		if (counter == 1) intervalCompensation = (unsigned long)iter.value().toInt();
		counter++;
	}
	
	return 1;
 }