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
bool notFirstRun = false;
int readingsToCollate = 3;

void initializeSensors();
JSONBufferWriter getSensorReadings(JSONBufferWriter writerData);
void qwiicTestForConnectivity();
void qwiicGetValue();
JSONBufferWriter readSPS30(JSONBufferWriter writerData);
void ErrtoMess(char *mess, uint8_t r);
void goSleep();
void syncClock();

// setup() runs once, when the device is first turned on.
void setup() {
	Particle.connect();
	pinMode(D7,OUTPUT);
	Wire.begin();
	Serial.begin();
	notFirstRun = false;
	initializeSensors();

	// Cloud sync initialization
	waitUntil(Particle.connected);
	Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));
	Particle.process();
	syncClock();

	// Wait for background tasks to finish, and SCD30 to start up
	delay(60s);

	// Turn off connectivity
	Particle.disconnect();
	waitUntil(Particle.disconnected);
	WiFi.off();
}

// loop() runs over and over again, as quickly as it can execute. Main Program flow goes here!
void loop() {
	// Start of JSON string
	char *dataJson = (char *) malloc(1050);
	JSONBufferWriter writerData(dataJson, 1049);
	writerData.beginObject();

	for (int collateCount = 0; collateCount < readingsToCollate; collateCount++){
		// Sleep until next sensor reading timing
		if (notFirstRun) {
			sleepConfig.mode(SystemSleepMode::ULTRA_LOW_POWER)
						.duration(sensingInterval + timeNow - Time.now());
			System.sleep(sleepConfig);
		}

		// Collate readings into 1 JSON string
		digitalWrite(D7,HIGH);
		String readingName = String::format("r%i", collateCount + 1);
		writerData.name(readingName).beginObject();
			timeNow = Time.now();
			writerData.name("TS").value(Time.format(timeNow, TIME_FORMAT_ISO8601_FULL));
			writerData = getSensorReadings(writerData);
		writerData.endObject();
		notFirstRun = true;
		digitalWrite(D7,LOW);
	}

	// End of JSON string
	writerData.endObject();
	writerData.buffer()[std::min(writerData.bufferSize(), writerData.dataSize())] = 0;

	// Wake up connectivity
	digitalWrite(D7,HIGH);
	Serial.begin();
	WiFi.on();
	Particle.connect();
	waitUntil(Particle.connected);

	// Publish collated JSON string
	Serial.println("Collated:");
	Serial.println(writerData.dataSize());
	Serial.println(dataJson);
	Serial.println("");
	if (!Particle.connected()) Particle.connect();
	waitUntil(Particle.connected);
	Particle.publish("sensor-readings", dataJson);

	// Sync device clock daily
	Particle.process();
  	syncClock();

	// Shut down connectivity
	Particle.disconnect();
	waitUntil(Particle.disconnected);
	WiFi.off();
	free(dataJson);
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
	Planned Data Structure:
	{
		"read1": 
		{
			"Timestamp1": "DateTime"
			"Measurement1": Value1
			"Measurement2": Value2
		}
	}
	*/

	// LUX Sensor (BH1750)
	bh.make_forced_measurement();
	writerData.name("lux").value(bh.get_light_level());

	// CO2 Sensor (SCD30)
	if (airSensor.dataAvailable()) {
		writerData.name("CO2-ppm").value(airSensor.getCO2());
		writerData.name("RH1%").value(airSensor.getHumidity());
		writerData.name("Temp1C").value(airSensor.getTemperature());
	}

	// Particulate Sensor (SPS30)
	sps30.wakeup();
	sps30.start();
	delay(30s);
	writerData = readSPS30(writerData);
	sps30.sleep();

	// Peak Sound Sensor (SPARKFUN SEN-15892)
	qwiicGetValue();
	writerData.name("ADC").value(ADC_VALUE);
	writerData.name("dB").value(dBnumber);

	// UV Sensor (VEML 6070)
	writerData.name("UV").value(uv.readUV());

	// Pressure, Temperature, Humidity Sensor (BME280)
	writerData.name("P-mbar").value(bme.readPressure()/100.0F);
	writerData.name("RH2%").value(bme.readHumidity());
	writerData.name("Temp2C").value(bme.readTemperature());

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
	uint8_t ret, error_cnt = 0;
	struct sps_values val;

	// loop to get data
	do {

	ret = sps30.GetValues(&val);

	// data might not have been ready
	if (ret == SPS30_ERR_DATALENGTH){

		if (error_cnt++ > 3) {
			ErrtoMess((char *) "Error during reading values: ",ret);
			return writerData;
		}
		delay(1000);
	}

	// if other error
	else if(ret != SPS30_ERR_OK) {
		ErrtoMess((char *) "Error during reading values: ",ret);
		return writerData;
	}

	} while (ret != SPS30_ERR_OK);

	writerData.name("PM1").value(val.MassPM1);
	writerData.name("PM2.5").value(val.MassPM2);
	writerData.name("PM4").value(val.MassPM4);
	writerData.name("PM10").value(val.MassPM10);
	return writerData;
}

void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
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