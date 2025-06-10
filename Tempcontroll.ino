#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1_bc.h>

// === Pin Definitions ===
#define ONE_WIRE_BUS 2  // DS18B20 data pin
#define PWM_PIN 5       // PWM output for Peltier

// === OneWire & Temperature Sensor Setup ===
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// === LCD Display (I2C Address: 0x27 or 0x3F) ===
LiquidCrystal_I2C lcd(0x27, 20, 4); // Try 0x3F if 0x27 doesn’t work

// === PID Variables ===
double Setpoint = 5.0;  // Default target temperature
double Input, Output;
double Kp = 30.0, Ki = 5.0, Kd = 0.0; // Adjust PID gains

// === Mode Selection (false = cooling, true = heating) ===
bool heatingMode = false;  
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

void setup() {
  Serial.begin(115200);
  sensors.begin();
  lcd.init();
  lcd.backlight();
  pinMode(PWM_PIN, OUTPUT);

  // === Configure Timer3 for 25 kHz PWM on Pin 5 ===
  TCCR3A = (1 << COM3A1) | (1 << WGM31);
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS30);
  ICR3 = 639;  // Set TOP value for 25 kHz
  OCR3A = 300;   // Start with 0 PWM (off)

  // === Setup PID ===
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, heatingMode ? 120 : 180);  // Limit PWM for heating
}

// === Function to Switch Between Cooling & Heating ===
void setPIDMode(bool heatMode) {
  heatingMode = heatMode;
  myPID.SetControllerDirection(heatingMode ? DIRECT : REVERSE);
  myPID.SetOutputLimits(0, heatingMode ? 120 : 180);
  Serial.println(heatingMode ? "HEATING MODE ON" : "COOLING MODE ON");
}

void loop() {
  sensors.requestTemperatures();

  // === Read Temperature ===
  float tempSum = 0;
  int validSensors = 0;
  int sensorCount = sensors.getDeviceCount();

  for (int i = 0; i < sensorCount; i++) {
    float tempC = sensors.getTempCByIndex(i);
    if (tempC != DEVICE_DISCONNECTED_C) {
      tempSum += tempC;
      validSensors++;
    }
  }
  Input = (validSensors > 0) ? (tempSum / validSensors) : -999;  // Error handling


  // === Compute PID Output ===
  myPID.Compute();
  OCR3A = map(Output, 0, heatingMode ? 120 : 180, 0, ICR3);

  // === Serial Communication ===
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove whitespace

    if (command.startsWith("SET ")) {
      double newTemp = command.substring(4).toFloat();
      if (newTemp >= 5 && newTemp <= 40) {  // Safety limits for temp
        Setpoint = newTemp;
        Serial.print("New Setpoint: ");
        Serial.println(Setpoint);
      } else {
        Serial.println("ERROR: Setpoint out of range (5 to 40°C)");
      }
    } else if (command == "H") {
      setPIDMode(true);
    } else if (command == "C") {
      setPIDMode(false);
    }
  }

  // === Serial Monitor Output ===
  Serial.print("Temp: ");
  Serial.print(Input);
  Serial.print(" C | Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" C | PWM: ");
  Serial.println(map(Output,0,255,0,100));

  // === Update LCD Display ===
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(Input);
  lcd.print(" C  ");

  lcd.setCursor(0, 1);
  lcd.print("Set: ");
  lcd.print(Setpoint);
  lcd.print(" C  ");

  lcd.setCursor(0, 2);
  lcd.print("PWM: ");
  lcd.print(map(Output,0,255,0,100));
  lcd.print("      "); // Clear extra chars

  delay(500);
}
