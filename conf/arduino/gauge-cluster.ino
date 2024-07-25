#include <ArduinoJson.h>
#include <DFL168A.h>
#include <Ewma.h>
#include <Adafruit_AHTX0.h>
#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>

/*
  Reset the Arduino board.
  Declare reset fuction at address 0.
*/
void(* resetFunc) (void) = 0;
/*
  Setup the Dafulai board to read the J1708 ECM data.
*/
DFL168A myDFL168A(&Serial1, J1708_PROTOCOL, 5000, 500000, 9600, 500);

/* 
  Exponentially Weighted Moving Average filter is used for smoothing data series readings.
  Ewma adcFilter1(0.1); //Less smoothing - faster to detect changes, but more prone to noise
  Ewma adcFilter2(0.01);  // More smoothing - less prone to noise, but slower to detect changes
*/
Ewma adcFilter(0.1);
Ewma adcFilterOilPressure(0.1);



/*
  Constant integer to set the baud rate for serial monitor
*/
const int BAUD_RATE = 9600;
/* 
  Analog to Digital resolution for Arduino
*/
const float A_2_D_RESOLUTION = 1024.0;
/* 
  Dafulai board J1708
*/
bool Vehicle_OK;
/* 
  Analog input pin for fuel tank sensor.
*/
const int ANALOG_PIN_FUEL_TANK = A2;
/* 
  Analog input pin for engine oil pressure sensor.
*/
const int ANALOG_PIN_ENGINE_OIL_PRESSURE = A3;


/** AIR PRESSURE TANKS - START **/
/*
  Primary Tank (green). Analog input pin for the pressure transducer 1.
*/
const int ANALOG_PIN_PRIMARY_AIR_TANK = A0;
/*
  Secondary Tank (red). Analog input pin for the pressure transducer 2.
*/
const int ANALOG_PIN_SECONDARY_AIR_TANK = A1;
/*
  Analog reading of pressure transducer at 0psi = 0.5v
*/
const int AIR_TANK_ANALOG_PRESSURE_ZERO = 102;
/*
  Analog reading of pressure transducer at 150psi = 4.5v
*/
const int AIR_TANK_ANALOG_PRESSURE_MAX = 921;
/*
  Max PSI read by transducer.
*/
const int AIR_TANK_MAX_PRESSURE = 150;
/** AIR PRESSURE TANKS - END **/

/*
  Variable to account for non blocking delays.
*/
unsigned long previousMillis = 0;
/*
  Constant to execute loop using this interval in millis.
*/
const long WAIT_INTERVAL = 1000;

/* 
  External Temp/Humidity.
  Adafruit AM2315C Sensor (AHT20).
*/
Adafruit_AHTX0 adafruitHumidityTempExternal;
/* 
  Adafruit AM2315C Sensor (AHT20)
*/
bool adafruitSensorAHTX0_OK;
/* 
  Internal Temp/Humidity/Pressure
  Adafruit MS8607 Sensor.
*/
Adafruit_MS8607 adafruitHumidityTempInternal;
/* 
  Adafruit MS8607 Sensor.
*/
bool adafruitSensorMS8607_OK;
/* 
  Adafruit Sensors Enum.
*/
sensors_event_t humidity, temp, pressure;

void setup() {
  Serial.begin(BAUD_RATE);
  //Setup Dafulai ECM board pin.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Start Dafulai board
  Vehicle_OK = myDFL168A.begin();
  if (Vehicle_OK) {
    digitalWrite(13, HIGH);
  } else {
    // Delay 3 seconds before reset the Arduino board.
    // Serial.println("ECM module not ready.");
    // Serial.println("Calling reset.");
    // delay(3000);
    // resetFunc();
  }
  // Setup external Humidity and Temperature sensors
  adafruitSensorAHTX0_OK = adafruitHumidityTempExternal.begin();
  adafruitSensorMS8607_OK = adafruitHumidityTempInternal.begin();
}

void loop() {
  // Get the current millis to execute the loop only every WAIT_INTERVAL period.
  unsigned long currentMillis = millis();
  // If WAIT_INTERVAL period is passed execute calls.
  if (currentMillis - previousMillis >= WAIT_INTERVAL) {
    // Update previousMillis variable.
    previousMillis = currentMillis;
    //Initiate Json Document
    JsonDocument docJson;
    // Read all data
    readAirTanks(docJson);
    readFuelLevel(docJson);
    readEngineOilPressure(docJson);
    // Only read external Temperature and Humidity if module is connected.
    if (adafruitSensorAHTX0_OK) {
      docJson["externalTempHumidity"] = true;
      readExternalTempHumidity(docJson);
    } else {
      docJson["externalTempHumidity"] = false;
    }
    // Only read internal Temperature, Humidity and Pressure if module is connected.
    if (adafruitSensorMS8607_OK) {
      docJson["internalTempHumidityPressure"] = true;
      readInternalTempHumidityPressure(docJson);
    } else {
      docJson["internalTempHumidityPressure"] = false;
    }

    // Only read J1708 data if module is connected to vehicle.    
    if (Vehicle_OK) {
      docJson["ecmData"] = true;
      readJ1708Data(docJson);
    } else {
      docJson["ecmData"] = false;
    }
    // Send Json over Serial.
    serializeJson(docJson, Serial);
    Serial.println("");
  }
}

/* Read Internal Temperature, Humidity and Pressure from Sensors */
void readInternalTempHumidityPressure(JsonDocument& doc) {
  adafruitHumidityTempInternal.getEvent(&pressure, &temp, &humidity);

  int t = celciusToFahrenheit(int(temp.temperature));
  doc["internalTemperature"] = t;
  doc["internalHumidity"] = int(humidity.relative_humidity);
  doc["internalPressure"] = int(pressure.pressure);
}

/* Read External Temperature and Humidity from Sensors */
void readExternalTempHumidity(JsonDocument& doc) {
//  sensors_event_t humidity, temp;
  adafruitHumidityTempExternal.getEvent(&humidity, &temp);

  int t = celciusToFahrenheit(int(temp.temperature));
  doc["externalTemperature"] = t;
  doc["externalHumidity"] = int(humidity.relative_humidity);
}

/* Read Engine Oil Pressure */
void readEngineOilPressure(JsonDocument& doc) {
   // Clean analogRead();
  int dummy = analogRead(ANALOG_PIN_ENGINE_OIL_PRESSURE);
  // Read the analog data from sensor.
  int val = 0;
  for(int i=0; i <10; i++) {
    val += analogRead(ANALOG_PIN_ENGINE_OIL_PRESSURE);
  }
  // Rounding for integer
  val += 5;
  //Find average
  val /=10;
  //Constrain readings for upper and lower limits
  val = constrain(val, 197, 598);
  // Calculate real input voltage based on analog reading.
  float inputVoltage = (val * 5.0) / A_2_D_RESOLUTION;
  //Oil Pressure equation (x = inputVoltage):
  // P = 51.2*x + (-49.4)
  float oilPressure = 51.2 * inputVoltage + (-49.4);
  // Cast and round float to integer.
  int totalPercentPressure = int(round(oilPressure));
  // Write Oil Pressure 0-100PSI to Json Document.
  doc["engineOilPressure"] = totalPercentPressure;
}

/* Read Fuel Level */
void readFuelLevel(JsonDocument& doc) {
  // Clean analogRead();
  int dummy = analogRead(ANALOG_PIN_FUEL_TANK);
  // Read the analog data from sensor.
  int val = 0;
  for(int i=0; i <10; i++) {
    val += analogRead(ANALOG_PIN_FUEL_TANK);
  }
  // Rounding for integer
  val += 5;
  //Find average
  val /=10;
  //Constrain readings for upper and lower limits
  val = constrain(val, 28, 760);
  // Calculate real input voltage based on analog reading.
  float inputVoltage = (val * 5.0) / A_2_D_RESOLUTION; 
  //Fuel Level equation (x = inputVoltage):
  // F = -0.381 + 10.6*x + 4.44*x^2
  float fuelLevel = -0.381 + 10.6*inputVoltage + 4.44*(inputVoltage * inputVoltage);
  // Cast and round float to integer.
  int totalPercentFuel = int(round(fuelLevel));
  // Write fuel level 0-100% to Json Document.
  doc["fuelLevel"] = totalPercentFuel;
}

/* Read Primary and Secondary Air Tank pressure from analog sensors */
void readAirTanks(JsonDocument& doc) {
  // Primary tank
  // Clean the analoRead()
  int dummy = analogRead(ANALOG_PIN_PRIMARY_AIR_TANK);
  int tempTankRead = 0;
  for(int i = 0; i < 10; i++ ) {
    tempTankRead += analogRead(ANALOG_PIN_PRIMARY_AIR_TANK);
  }
  // Allows rounding for integer math.
  tempTankRead += 5;
  // Get the average for the 10 readings.
  float pressureValue = tempTankRead / 10;
  //Constrain readings for upper and lower limits
  tempTankRead = constrain(pressureValue, AIR_TANK_ANALOG_PRESSURE_ZERO, AIR_TANK_ANALOG_PRESSURE_MAX);
  // Map AnalogRead value to the PSI range 0-150psi
  tempTankRead = map(tempTankRead, AIR_TANK_ANALOG_PRESSURE_ZERO, AIR_TANK_ANALOG_PRESSURE_MAX, 0, AIR_TANK_MAX_PRESSURE);
  // Add tank PSI to Json Document
  doc["primaryAirTank"] = tempTankRead;

  // Secondary tank
  // Clean the analoRead()
  dummy = analogRead(ANALOG_PIN_SECONDARY_AIR_TANK);
  tempTankRead = 0;
  for(int x = 0; x < 10; x++ ) {
    tempTankRead += analogRead(ANALOG_PIN_SECONDARY_AIR_TANK);
  }
  // Allows rounding for integer math.
  tempTankRead += 5;
  // Get the average for the 10 readings.
  pressureValue = tempTankRead / 10;
  //Constrain readings for upper and lower limits
  tempTankRead = constrain(pressureValue, AIR_TANK_ANALOG_PRESSURE_ZERO, AIR_TANK_ANALOG_PRESSURE_MAX);
  // Map AnalogRead value to the PSI range 0-150psi
  tempTankRead = map(tempTankRead, AIR_TANK_ANALOG_PRESSURE_ZERO, AIR_TANK_ANALOG_PRESSURE_MAX, 0, AIR_TANK_MAX_PRESSURE);
  // Add tank PSI to Json Document
  doc["secondaryAirTank"] = tempTankRead;
}

/* Read all data from J1708 module and stores into JsonDocument */
void readJ1708Data(JsonDocument& doc) {
  //Current vehicle speed km/h.
  float VehicleSpeed;
  //It is the ratio of current output torque to maximum torque available at the current engine speed. This is percentage.
  float EngineLoad;
  //It is the temperature of liquid found in engine cooling system in Celsius degree.
  int  CoolantTemperature;
  //It is the current fuel economy at current vehicle velocity in Km/L.
  float InstantFuelEconomy;
  //It is the Average of instantaneous fuel economy for that segment of vehicle operation of interest in Km/L.
  float AvgFuelEconomy;
  //It is the net brake power that the engine will deliver continuously, specified for a given application at a rated speed in KW.
  float RatedEnginePower;
  //It is the measured electrical potential of the battery in Volts.
  float BatteryVoltage;
  //It is the temperature of air surrounding vehicle(engine bay?) in Celsius degree.
  int AmbientTemp;
  //It is the rotational velocity of crankshaft in RPM.
  int EngineSpeed;
  //It is the temperature of precombustion air found in intake manifold of engine air supply system in Celsius degree.
  int IntakeManifoldTemp;
  //It is the ratio of actual accelerator pedal position to maximum pedal position. This is percentage.
  float AccelPedalPosi1;
  //It is the amount of fuel consumed by engine per unit of time in L/s.
  float FuelRate;

  if (myDFL168A.J1708.getVehicleSpeed(VehicleSpeed)) {
    int mph = kmhToMph(VehicleSpeed);
    doc["vehicleSpeed"] = mph;
  } else {
    doc["vehicleSpeed"] = NULL;
  }

  if (myDFL168A.J1708.getEngineLoad(EngineLoad)) {
    doc["engineLoad"] = int(EngineLoad);
  } else {
    doc["engineLoad"] = NULL;
  }

    if (myDFL168A.J1708.getCoolantTemperature(CoolantTemperature)) {
    int far = celciusToFahrenheit(CoolantTemperature);
    doc["coolantTemperature"] = far;
  } else {
    doc["coolantTemperature"] = NULL;
  }

  if (myDFL168A.J1708.getInstantFuelEconomy(InstantFuelEconomy)) {
    doc["instantFuelEconomy"] = InstantFuelEconomy;
  } else {
    doc["instantFuelEconomy"] = NULL;
  }

  if (myDFL168A.J1708.getAvgFuelEconomy(AvgFuelEconomy)) {
    float mpg = kmlToMpg(AvgFuelEconomy);
    doc["avgFuelEconomy"] = mpg;
  } else {
    doc["avgFuelEconomy"] = NULL;
  }

  if (myDFL168A.J1708.getRatedEnginePower(RatedEnginePower)) {
    float hp = kwToHp(RatedEnginePower);
    doc["ratedEnginePower"] = hp;
  } else {
    doc["ratedEnginePower"] = NULL;
  }

  if (myDFL168A.J1708.getBatteryVoltage(BatteryVoltage)) {
    doc["batteryVoltage"] = BatteryVoltage;
  } else {
    doc["batteryVoltage"] = NULL;
  }

  if (myDFL168A.J1708.getAmbientTemp(AmbientTemp)) {
    int fah = celciusToFahrenheit(AmbientTemp);
    doc["ambientTempEngineBay"] = fah;
  } else {
    doc["ambientTempEngineBay"] = NULL;
  }

  if (myDFL168A.J1708.getEngineSpeed(EngineSpeed)) {
    doc["rpm"] = EngineSpeed;
  } else {
    doc["rpm"] = NULL;
  }

  if (myDFL168A.J1708.getIntakeManifoldTemp(IntakeManifoldTemp)) {
    int fah = celciusToFahrenheit(IntakeManifoldTemp);
    doc["intakeManifoldAirTemp"] = fah;
  } else {
    doc["intakeManifoldAirTemp"] = NULL;
  }

  if (myDFL168A.J1708.getAccelPedalPosi1(AccelPedalPosi1)) {
    doc["accelPedalPosi1"] = int(AccelPedalPosi1);
  } else {
    doc["accelPedalPosi1"] = NULL;
  }

  if (myDFL168A.J1708.getFuelRate(FuelRate)) {
    doc["fuelRate"] = FuelRate;
  } else {
    doc["fuelRate"] = NULL;
  }
}

// Convert Kilometer per Hour to Miles per Hour
int kmhToMph(float kmh) {
  int mph = kmh * 0.6213711922;
  return mph;
}
// Convert Kilometer per Liter to Miles per Gallon
float kmlToMpg(float kml) {
  float mpg = kml * 2.35215;
  return mpg;
}
// Convert Celcius to Fahrenheit
int celciusToFahrenheit(int celsius) {
  int fahrenheit = ((celsius * 9) / 5 + 32);
  return fahrenheit;
}
// Converts Kilowatt to Mechanical Horsepower
float kwToHp(float kw) {
  float hp = kw * 1.341;
  return hp;
}
