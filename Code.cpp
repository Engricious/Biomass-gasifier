/* 
  Biomass Gasifier Controller (ESP32)
  - Reads K-type thermocouple via MAX31855 (SPI)
  - Reads analog pressure sensor on A0
  - Controls blower via PWM (LEDC)
  - Controls relays for safety/valve
  - Simple PID blower control to maintain target temperature
  - Safety shutdown on over-temp or over-pressure
  - Optional WiFi telemetry (commented / placeholder)
  
  Libraries required:
   - Adafruit MAX31855 (or MAX6675 equivalent)
     Install via Library Manager: "Adafruit MAX31855 library" or "MAX6675"
*/

#include <SPI.h>
#include <Adafruit_MAX31855.h> // or use MAX6675 library as needed
//#include <WiFi.h>     // Uncomment if using WiFi telemetry
//#include <HTTPClient.h>

//////////////////////////////////////
// Pin definitions (edit as needed) //
//////////////////////////////////////

// MAX31855 thermocouple SPI pins (example using VSPI)
const int TCK_SCK  = 18; // SCK
const int TCK_CS   = 5;  // CS / CS0
const int TCK_SO   = 19; // MISO (SO)
Adafruit_MAX31855 thermocouple(TCK_SCK, TCK_CS, TCK_SO); // constructor variant may differ per library

// ADC (pressure sensor)
const int PRESSURE_PIN = 36; // VP (A0) on many ESP32 boards

// Blower PWM (LEDC)
const int BLOWER_PIN = 25;
const int BLOWER_FREQ = 20000; // 20 kHz typical
const int BLOWER_CHANNEL = 0;
const int BLOWER_RESOLUTION = 8; // 8-bit PWM (0-255)

// Relays / actuators
const int RELAY_VALVE = 26;    // solenoid valve control
const int RELAY_GEN   = 27;    // generator or fuel cutoff relay

// Status LED
const int LED_PIN = 2;

/////////////////////////
// Control parameters  //
/////////////////////////

float targetTempC = 850.0;   // desired reactor setpoint (degC) - adjust for your fuel
float tempHysteresis = 10.0; // hysteresis for simple on/off fallback
int pwmMin = 10;  // minimum blower PWM (safe idle)
int pwmMax = 255; // full speed

// Pressure thresholds (units: sensor voltage -> convert to kPa or desired)
float pressureSafeMax = 1.0; // example, in volts - calibrate to your sensor
float pressureShutdown = 2.5; // example shutdown threshold (V)

// Safety temperatures
float tempShutdown = 1000.0; // if exceeded, immediate shutdown
float tempWarn = 900.0;      // warning level (log/LED)

// PID coefficients (simple)
float Kp = 0.14;
float Ki = 0.004;
float Kd = 0.0;

//////////////////////
// Runtime storage  //
//////////////////////
float integralTerm = 0.0;
float lastTempError = 0.0;
unsigned long lastPIDMillis = 0;
unsigned long pidInterval = 1000; // ms

bool systemEnabled = true;
bool emergencyShutdown = false;

/////////////////////
// Helper functions //
/////////////////////

// map float value from one range to another
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// read pressure sensor and convert to voltage (if ADC returns raw)
float readPressureVoltage() {
  int raw = analogRead(PRESSURE_PIN); // 0..4095 on ESP32 for 12-bit ADC
  // Convert to 0..3.3V (adjust if using attenuation)
  float voltage = (raw / 4095.0) * 3.3;
  return voltage;
}

// emergency shutdown routine
void doEmergencyShutdown(const char* reason) {
  Serial.print("EMERGENCY SHUTDOWN: ");
  Serial.println(reason);
  emergencyShutdown = true;
  systemEnabled = false;
  // Turn off blower
  ledcWrite(BLOWER_CHANNEL, 0);
  // Close valve / disable generator
  digitalWrite(RELAY_VALVE, LOW);
  digitalWrite(RELAY_GEN, LOW);
  // Blink LED rapidly
  for (int i=0;i<5;i++){
    digitalWrite(LED_PIN, HIGH);
    delay(120);
    digitalWrite(LED_PIN, LOW);
    delay(120);
  }
}

// basic logger (Serial)
void logTelemetry(float tempC, float pressureV, int blowerPwm) {
  Serial.print("Temp(C): "); Serial.print(tempC, 1);
  Serial.print(" | Pressure(V): "); Serial.print(pressureV, 3);
  Serial.print(" | PWM: "); Serial.println(blowerPwm);
}

//////////////////////
// Setup & loop     //
//////////////////////

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize pins
  pinMode(RELAY_VALVE, OUTPUT);
  pinMode(RELAY_GEN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_VALVE, LOW);
  digitalWrite(RELAY_GEN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Setup PWM (LEDC)
  ledcSetup(BLOWER_CHANNEL, BLOWER_FREQ, BLOWER_RESOLUTION);
  ledcAttachPin(BLOWER_PIN, BLOWER_CHANNEL);
  ledcWrite(BLOWER_CHANNEL, 0);

  // thermocouple begin if needed (the Adafruit library does initialization in constructor)
  // verify thermocouple presence
  double t = thermocouple.readCelsius();
  Serial.print("Thermocouple initial read (C): ");
  Serial.println(t);

  lastPIDMillis = millis();

  Serial.println("System ready.");
}

void loop() {
  if (emergencyShutdown) {
    delay(1000);
    return;
  }

  // Read sensors
  double tempC = thermocouple.readCelsius(); // returns NAN if error
  if (isnan(tempC)) {
    Serial.println("Thermocouple read error (NAN). Check wiring.");
    // handle as needed (e.g., stop system)
    // For now, we'll keep running but warn
  }

  float pressureV = readPressureVoltage();

  // Safety checks
  if (tempC >= tempShutdown) {
    doEmergencyShutdown("Temperature exceeded shutdown threshold");
    return;
  }
  if (pressureV >= pressureShutdown) {
    doEmergencyShutdown("Pressure exceeded shutdown threshold");
    return;
  }

  // Warning LED if temp near warn zone
  if (tempC >= tempWarn) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // Simple PID controller every pidInterval ms
  unsigned long now = millis();
  if (now - lastPIDMillis >= pidInterval) {
    float error = targetTempC - (float)tempC;
    integralTerm += error * (pidInterval / 1000.0f);
    float derivative = (error - lastTempError) / (pidInterval / 1000.0f);

    float pidOutput = (Kp * error) + (Ki * integralTerm) + (Kd * derivative);
    lastTempError = error;
    lastPIDMillis = now;

    // Convert pidOutput to PWM
    float pwmFloat = fmap(pidOutput, -200.0, 200.0, pwmMin, pwmMax);
    int pwm = constrain((int)pwmFloat, 0, 255);

    // Safety: if pressure too high, reduce blower
    if (pressureV > pressureSafeMax) {
      pwm = max(pwmMin, (int)(pwm * 0.6)); // reduce to 60% of computed
      Serial.println("Pressure above safe threshold, reducing blower power.");
    }

    // Update blower PWM
    ledcWrite(BLOWER_CHANNEL, pwm);

    // Open/close valve based on some simple rules (example)
    if (tempC < (targetTempC - tempHysteresis)) {
      digitalWrite(RELAY_VALVE, HIGH); // open valve to feed air
    } else if (tempC > (targetTempC + tempHysteresis)) {
      digitalWrite(RELAY_VALVE, LOW); // close valve / throttle
    }

    // Telemetry
    logTelemetry((float)tempC, pressureV, pwm);
  }

  // Small delay to avoid tight loop
  delay(200);
}
