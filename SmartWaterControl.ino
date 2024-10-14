#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin assignments
#define SENSOR1_PIN D8          // First water sensor connected to D8
#define SENSOR2_PIN D6          // Second water sensor connected to D6
#define RELAY_PIN_SOLENOID D4   // Relay for solenoid connected to D4 (Active LOW)
#define RELAY_OUTPUT_SOLENOID D5 // Relay for output solenoid connected to pin 10 (SD2)
#define ONE_WIRE_BUS_1 D7       // DS18B20 for tank 1 connected to D5
#define LDR_PIN A0              // LDR connected to A0

// Setup OneWire and DS18B20
OneWire oneWire1(ONE_WIRE_BUS_1);  // Tank 1 temperature sensor
DallasTemperature sensorsTank1(&oneWire1);

// Initialize the I2C LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // Adjust the address (0x27) and size (16, 2) as needed

void setup() {
  Serial.begin(115200);

  // Initialize the LCD
  lcd.begin();
  lcd.backlight();

  // Display initialization message
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  Serial.println("Initializing...");

  // Simulate a delay for initialization
  delay(2000);

  // Display system start message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");
  Serial.println("System Starting...");

  // Configure sensor pins as inputs
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);

  // Configure relay pins as outputs
  pinMode(RELAY_PIN_SOLENOID, OUTPUT);
  pinMode(RELAY_OUTPUT_SOLENOID, OUTPUT);

  // Initially turn off both relays (HIGH to keep them OFF for active LOW relays)
  digitalWrite(RELAY_PIN_SOLENOID, HIGH);
  digitalWrite(RELAY_OUTPUT_SOLENOID, HIGH);

  Serial.println("Relays OFF at startup");

  // Start the DS18B20 temperature sensor
  sensorsTank1.begin();

  // Wait a bit before the system begins operation
  delay(2000);

  // Clear the display after system initialization
  lcd.clear();
}

void loop() {
  // Read temperature from DS18B20
  sensorsTank1.requestTemperatures();  // Get temperature for tank 1

  float tempTank1 = sensorsTank1.getTempCByIndex(0);  // Get tank 1 temperature in Celsius

  // Read states of the two water sensors
  int sensor1State = digitalRead(SENSOR1_PIN);  // Read state of sensor 1
  int sensor2State = digitalRead(SENSOR2_PIN);  // Read state of sensor 2

  // Read LDR value
  int ldrValue = analogRead(LDR_PIN);  // Read LDR value
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);

  // Threshold for LDR (Adjust this based on your lighting condition)
  int ldrThreshold = 600;

  // Print sensor states and temperature for debugging
  Serial.print("Sensor 1: ");
  Serial.print(sensor1State == LOW ? "EMPTY" : "FULL");
  Serial.print(" | Sensor 2: ");
  Serial.print(sensor2State == LOW ? "EMPTY" : "FULL");
  Serial.print(" | TempTank1: ");
  Serial.print(tempTank1);
  Serial.println(" °C");

  // Check for daylight
  if (ldrValue >= ldrThreshold) {
    // Daytime detected - keep solenoid relay ON
    digitalWrite(RELAY_PIN_SOLENOID, LOW);  // Turn ON solenoid (active LOW)
    Serial.println("Daytime detected - Solenoid Relay ON");
  } else {
    // Nighttime or no sunlight detected, control solenoid based on sensors
    Serial.println("No sunlight detected - checking water sensors.");

    // Control solenoid relay based on water level sensors
    if (sensor1State == LOW && sensor2State == LOW) {
      digitalWrite(RELAY_PIN_SOLENOID, LOW);  // Turn ON solenoid (active LOW)
      Serial.println("Both sensors EMPTY - Solenoid Relay ON");
    } else if (sensor1State == HIGH && sensor2State == HIGH) {
      digitalWrite(RELAY_PIN_SOLENOID, HIGH);  // Turn OFF solenoid
      Serial.println("Both sensors FULL - Solenoid Relay OFF");
    }
  }

  // Control output solenoid relay based on tank 1 temperature
  if (tempTank1 < 45) {
    digitalWrite(RELAY_OUTPUT_SOLENOID, LOW);  // Turn ON output solenoid (active LOW)
    Serial.println("Tank 1 Temp below 45°C - Output Solenoid ON");
  } else {
    digitalWrite(RELAY_OUTPUT_SOLENOID, HIGH);  // Turn OFF output solenoid
    Serial.println("Tank 1 Temp above 45°C - Output Solenoid OFF");
  }

  // Update the LCD with temperature and output solenoid status
  lcd.clear();
  lcd.setCursor(0, 0); // Set cursor to first row
  lcd.print("Temp: ");
  lcd.print(tempTank1);
  lcd.print("C");
  
  lcd.setCursor(0, 1); // Set cursor to second row
  lcd.print("OutSol: ");
  lcd.print(digitalRead(RELAY_OUTPUT_SOLENOID) == HIGH ? "OFF" : "ON");

  delay(500);  // Wait for 500 milliseconds before checking again
}
