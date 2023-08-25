#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <AFMotor.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define IR_SENSOR1_PIN A0
#define IR_SENSOR2_PIN A1

#define BUZZER_PIN 5

#define GSM_TX_PIN 12
#define GSM_RX_PIN 13

#define LCD_ADDRESS 0x3F
#define LCD_COLUMNS 16
#define LCD_ROWS 2

SoftwareSerial gsmSerial(GSM_TX_PIN, GSM_RX_PIN); // GSM module SoftwareSerial
TinyGPSPlus gps; // Create a TinyGPS++ object

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS); // LCD object

AF_DCMotor motor1(1); // Motor 1
AF_DCMotor motor2(2); // Motor 2
AF_DCMotor motor3(3); // Motor 3
AF_DCMotor motor4(4); // Motor 4

int irThreshold1 = 800; // Threshold for crack detection using IR sensor 1
int irThreshold2 = 800; // Threshold for crack detection using IR sensor 2

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_SENSOR1_PIN, INPUT);
  pinMode(IR_SENSOR2_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(9600);
  gsmSerial.begin(9600);

  Wire.begin(); // Initialize I2C communication
  lcd.begin(LCD_COLUMNS, LCD_ROWS); // Initialize the LCD display

  // Set motor speeds
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);

  // Initialize GSM module
  sendGSMCommand("AT");
  sendGSMCommand("AT+CMGF=1"); // Set SMS text mode
  sendGSMCommand("AT+CNMI=1,2,0,0,0"); // Enable new SMS message notifications

  lcd.backlight();
  lcd.clear();
}

void loop() {
  // Obstacle detection
  float distance = getDistance();
  if (distance < 40) {
    displayMessage("Obstacle Detected!");
    stopMotors();
    activateBuzzer();
    delay(2000);
    deactivateBuzzer();
  } else {
    moveForward();
  }

  // Print obstacle distance on LCD
  lcd.setCursor(0, 1);
  lcd.print("Distance: ");
  lcd.print(distance);
  lcd.print(" cm");

  // Crack detection using IR sensors
  int irSensor1Value = analogRead(IR_SENSOR1_PIN);
  int irSensor2Value = analogRead(IR_SENSOR2_PIN);

  if (irSensor1Value < irThreshold1) {
    if (irSensor1Value < minorCrackThreshold) {
      displayMessage("Minor Crack Detected - Sensor 1");
    } else {
      displayMessage("Major Crack Detected - Sensor 1");
    }
    stopMotors();
    activateBuzzer();
    delay(2000);
    deactivateBuzzer();
  }

  if (irSensor2Value < irThreshold2) {
    if (irSensor2Value < minorCrackThreshold) {
      displayMessage("Minor Crack Detected - Sensor 2");
    } else {
      displayMessage("Major Crack Detected - Sensor 2");
    }
    stopMotors();
    activateBuzzer();
    delay(2000);
    deactivateBuzzer();

  // GPS location
  while (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {
      if (gps.location.isValid()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        sendGPSLocation(latitude, longitude);
      }
    }
  }
}

float getDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration / 2) * 0.0343;

  return distance;
}

void sendGSMCommand(String command) {
  gsmSerial.println(command);
  delay(500);
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
}

void sendGPSLocation(float latitude, float longitude) {
  String message = "Latitude: " + String(latitude, 6) + ", Longitude: " + String(longitude, 6);
  sendMessage(message);
}

void sendMessage(String message) {
  sendGSMCommand("AT+CMGS=\"+1234567890\""); // Replace with recipient's phone number
  delay(1000);
  gsmSerial.print(message);
  delay(1000);
  gsmSerial.write(26); // End SMS transmission
  delay(100);
}

void displayMessage(String message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
  Serial.println(message);
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void activateBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void deactivateBuzzer() {
  digitalWrite(BUZZER_PIN, LOW);
}
