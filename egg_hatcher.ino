#define TEMP_SENSOR_PIN A0   // Define the analog pin for the temperature sensor
#define HEATING_PIN 2        // Define the digital pin for the heating element
#define SET_POINT 37.5      // Define the desired temperature setpoint in Celsius
#define TOLERANCE 0.5       // Define the acceptable temperature range in Celsius

// Define the PID parameters
double Kp = 4.0;      // Proportional gain
double Ki = 0.2;      // Integral gain
double Kd = 1.0;      // Derivative gain
double input, output, setpoint;
double error, last_error, integral, derivative;
unsigned long last_time;

void setup() {
  pinMode(HEATING_PIN, OUTPUT);
  Serial.begin(9600);    // Initialize serial communication for debugging
  input = getTemperature();   // Initialize the input value to the current temperature
  setpoint = SET_POINT;       // Initialize the setpoint
  last_time = millis();       // Initialize the time
}

void loop() {
  input = getTemperature();    // Read the temperature from the sensor
  error = setpoint - input;    // Compute the error
  unsigned long current_time = millis();    // Get the current time
  double dt = (current_time - last_time) / 1000.0;   // Compute the time difference in seconds
  integral += error * dt;     // Compute the integral
  derivative = (error - last_error) / dt;    // Compute the derivative
  output = Kp * error + Ki * integral + Kd * derivative;    // Compute the PID output
  last_error = error;         // Save the error for the next iteration
  last_time = current_time;   // Save the time for the next iteration

  Serial.print("Temperature: ");
  Serial.print(input);
  Serial.print(" C, Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" C, Output: ");
  Serial.print(output);
  Serial.println();

  if (output > 0) {
    digitalWrite(HEATING_PIN, HIGH);   // Turn on the heating element
    delay(output);                     // Delay for the on-time of the heating element
  }
  digitalWrite(HEATING_PIN, LOW);      // Turn off the heating element
  delay(1000 - output);                // Delay for the off-time of the heating element
}

float getTemperature() {
  float voltage = analogRead(TEMP_SENSOR_PIN) * 5.0 / 1024.0;   // Read the voltage from the sensor
  float temperature = voltage * 100.0;                           // Convert the voltage to temperature in Celsius
  return temperature;
}
