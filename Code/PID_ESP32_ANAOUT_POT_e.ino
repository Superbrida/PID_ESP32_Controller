#include <OneWire.h>
#include <DallasTemperature.h>

// Temperature sensor configuration
#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temperature;

// Analog output DAC configuration
#define OUTPUT_PIN 25 // modify the pin according to the board used

// Potentiometer configuration
#define POTENTIOMETER_PIN 34 // modify the pin according to the board used

// PID configuration
double setpoint = 22.0;
double Kp = 30.0;
double Ki = 15.0;
double Kd = 5.0;
double outputMin = 0.0;
double outputMax = 4095.0;
double integralTerm, lastError;
unsigned long lastTime;

void setup() {
  Serial.begin(9600);

  // Initialize the temperature sensor
  sensors.begin();

  // Initialize the analog output DAC
  //dac_output_enable(OUTPUT_PIN);

  // Set the initial values for accumulated error and previous error
  integralTerm = 0;
  lastError = 0;

  // Initialize the last sampling time
  lastTime = millis();
}

// ...
// ...

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(POTENTIOMETER_PIN);
  int mappedValue = map(potValue, 0, 4095, -300, 300); // Map the read value to the desired temperature range (-300 to +300 to get 0.1 steps)
  setpoint = (double) mappedValue / 10.0; // Convert the mapped value to degrees Celsius with 0.1 steps

  // Read temperature from the sensor
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);

  // Calculate the time elapsed since the last sampling
  unsigned long now = millis();
  double timeDelta = (double)(now - lastTime) / 1000.0;

  // Calculate the error
  double error = temperature - setpoint; // Invert the error sign for cooling

  // Add the error to the integral term
  integralTerm += error * Ki * timeDelta;

  // Limit the accumulated error within the range [outputMin, outputMax]
  integralTerm = constrain(integralTerm, outputMin, outputMax);

  // Calculate the derivative of the error
  double dError = (error - lastError) / timeDelta;

  // Calculate the PID output
  double output = Kp * error + integralTerm - Kd * dError;

  // Limit the PID output within the range [outputMin, outputMax]
  output = constrain(output, outputMin, outputMax);

  // Write the output to the DAC
  //dac_output_voltage(OUTPUT_PIN, output);
  analogWrite(OUTPUT_PIN, output / 4095.0 * 255);

  // Calculate the output value in percentage
  int outputPercentage = map(output, outputMin, outputMax, 0, 100);

  // Print values to Serial Monitor
  Serial.print(temperature);
  Serial.print(" C | Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" C | Output: ");
  Serial.print(output);
  Serial.print(" (");
  Serial.print(outputPercentage);
  Serial.println("%)");

  // Update the previous error and last sampling time
  lastError = error;
  lastTime = now;

  delay(1000);
}