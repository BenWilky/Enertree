// Required Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <NewPing.h>
#include <SimpleKalmanFilter.h>

// Pin out - Updated for ESP32
#define LED 2
#define FLOW_PIN 21             // GPIO 21
#define OVERFLOW_VALVE_PIN 23   // GPIO 23
#define SANITATION_VALVE_PIN 22 // GPIO 22
// #define UVLED_PIN 5
#define PUMP_PWM 19    // GPIO 19
#define IN1 18         // GPIO 18
#define IN2 5          // GPIO 5
#define TRIGGER_PIN 17 // GPIO 17
#define ECHO_PIN 16    // GPIO 16

// Constants
#define minFlowRate 0.2 // L/min
#define levelMax 5
#define levelMid 15
#define levelMin 30 // 30 is best

// Variables
int speed = 0;
float waterLevel = 0;
float avgWaterLevel = 0;
float prevLevel = 0;
int percentFull = 0;
bool isForced = false;
bool isOverflowing = false;
bool isSanitizing = false;
bool isRising = false;
bool isFlushing = false;
unsigned long flushInterval = 5 * 60 * 1000; // 60 * 1000; // 5 hours
unsigned long flushDuration = 30 * 1000;     //* 60 * 1000;     // 30 minutes
unsigned long flushStartTime = 0;
unsigned long lastFlowCheckTime = 0;
unsigned long levelCheckTime = 0;
unsigned long flushIntervalTime = 0;
unsigned long prevLevelCheckTime = 0;
unsigned long dashboardUpdateTime = 0;
unsigned int uS;
volatile int flowCount = 0;

// Wifi Credentials
const char *ssid = "BTW - Pro Max 13";
const char *password = "14881488";

// Dashboard and Server Setup
AsyncWebServer server(80);
ESPDash dashboard(&server);

// Dashboard Cards
Card waterHeigth1(&dashboard, GENERIC_CARD, "Raw Water Tank", "%");
Card waterHeigth2(&dashboard, GENERIC_CARD, "Sanitized Water Tank", "%");
Card overflow(&dashboard, BUTTON_CARD, "Force Overflow");
Card sanitize(&dashboard, BUTTON_CARD, "Force Sanitize");
Card pumpSpeed(&dashboard, SLIDER_CARD, "Pump Speed", "", 0, 255);
Card systemStatus(&dashboard, STATUS_CARD, "System Status", "idle");
Chart waterHeightChart(&dashboard, BAR_CHART, "Water Height Chart - Tank 1");

String XAxis[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"};
int YAxis[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Ultrasonic Sensor
NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200);

// Kalmann Filter
SimpleKalmanFilter kalmanFilter(1, 1, 0.01);

void flowISR()
{
  flowCount++;
}

int percentWaterLevel()
{
  return ((avgWaterLevel - levelMax) / (levelMin - levelMax) * 100);
}

void setup()
{
  // Serial Communication Setup
  Serial.begin(115200);

  // Pins Setup
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);
  pinMode(SANITATION_VALVE_PIN, OUTPUT);
  pinMode(OVERFLOW_VALVE_PIN, OUTPUT);
  // pinMode(UVLED_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(PUMP_PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LED, OUTPUT);

  // Initial Pin State
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(SANITATION_VALVE_PIN, HIGH);
  digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
  digitalWrite(LED, LOW);
  // digitalWrite(UVLED_PIN, HIGH);

  // Wifi Setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());
  server.begin();

  waterHeightChart.updateX(XAxis, 11);

  uS = sonar.ping_median(5);         // Send ping, get ping time in microseconds (uS).
  waterLevel = sonar.convert_cm(uS); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  avgWaterLevel = kalmanFilter.updateEstimate(waterLevel);
  prevLevel = avgWaterLevel;
}

void loop()
{
  if ((millis() - levelCheckTime) > 1000)
  {
    uS = sonar.ping_median(5);         // Send ping, get ping time in microseconds (uS).
    waterLevel = sonar.convert_cm(uS); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    avgWaterLevel = kalmanFilter.updateEstimate(waterLevel);
    if (millis() - prevLevelCheckTime > 0.5 * 60 * 1000)
    {
      prevLevel = avgWaterLevel;
      prevLevelCheckTime = millis();
    }
    Serial.print("Water level: ");
    Serial.print(avgWaterLevel);
    Serial.print(", Prev Level: ");
    Serial.println(prevLevel);
    levelCheckTime = millis();
  }

  // Is control forced by dashboard?
  if (isForced == true)
  {
    isOverflowing = false;
    isSanitizing = false;
    isRising = false;
    isFlushing = false;
    systemStatus.update("Forced Control", "danger");
    dashboard.sendUpdates();
  }

  // Update Dashboard
  if ((millis() - dashboardUpdateTime) > 1 * 60 * 1000)
  {
    // Insert the newest avgWaterLevel into the front of the array and shift the old values back
    percentFull = percentWaterLevel();
    for (int i = 10; i > 0; i--)
    {
      YAxis[i] = YAxis[i - 1];
    }
    YAxis[0] = percentFull;
    waterHeigth1.update(percentFull);
    waterHeightChart.updateY(YAxis, 11);
    dashboard.sendUpdates();
    dashboardUpdateTime = millis();
  }

  // Idle Mode
  if (avgWaterLevel < levelMin && avgWaterLevel > levelMax && !isFlushing && !isSanitizing && !isOverflowing && !isForced)
  {
    // Serial.println("Waiting Mode");
    digitalWrite(SANITATION_VALVE_PIN, HIGH);
    digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
    // digitalWrite(UVLED_PIN, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    systemStatus.update("Idle", "idle");
    dashboard.sendUpdates();
  }

  // Check if water level is rising or falling
  bool isCurrentlyRising = avgWaterLevel < prevLevel;
  if (isCurrentlyRising != isRising && !isRising && millis() > 5000 && !isForced) //&& (millis() - flushIntervalTime) > flushInterval)
  {
    // Water level has changed direction
    Serial.println("Flush beginning, opening overflow valve");
    isRising = isCurrentlyRising;
    isFlushing = true;
    digitalWrite(OVERFLOW_VALVE_PIN, LOW);
    flushStartTime = millis();
    flushIntervalTime = millis();
    systemStatus.update("Flushing", "warning");
    dashboard.sendUpdates();
  }

  // Check if overflow valve needs to be opened
  if (avgWaterLevel < levelMax && !isSanitizing && millis() > 5000 && !isForced)
  {
    if (!isOverflowing)
    {
      Serial.println("Overflow valve opened");
      isOverflowing = true;
      digitalWrite(OVERFLOW_VALVE_PIN, LOW);
      delay(10);
      isRising = false;
      systemStatus.update("Overflow", "danger");
      dashboard.sendUpdates();
    }
  }
  else if (avgWaterLevel > levelMid && isOverflowing && !isForced)
  {
    Serial.println("Overflow valve closed");
    isOverflowing = false;
    delay(10);
    digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
  }

  // Check if sanitation valve needs to be opened
  if ((millis() - flushStartTime) > flushDuration && isFlushing && !isForced)
  {
    if (!isSanitizing)
    {
      isFlushing = false;
      isOverflowing = false;
      digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
      Serial.println("Sanitation valve opened");
      isSanitizing = true;
      // digitalWrite(UVLED_PIN, LOW);
      delay(10);
      digitalWrite(SANITATION_VALVE_PIN, LOW);
      delay(10);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PUMP_PWM, speed);
      systemStatus.update("Sanitizing", "success");
      dashboard.sendUpdates();
    }
  }
  else if (avgWaterLevel >= levelMin)
  {
    if (isSanitizing)
    {
      Serial.println("Sanitation valve closed");
      isSanitizing = false;
      isRising = false;
      isFlushing = false;
      isOverflowing = false;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      delay(10);
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      delay(10);
      // digitalWrite(UVLED_PIN, HIGH);
    }
  }
  // Check flow rate
  if (millis() - lastFlowCheckTime >= 1000 && isSanitizing && !isForced)
  {
    lastFlowCheckTime = millis();
    float flowRate = (((float)flowCount) / 23);
    Serial.print("Flow rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");
    if (flowRate < minFlowRate && isSanitizing)
    {
      Serial.println("Flow rate too low, closing sanitation valve and turning off pump and UV LED");
      systemStatus.update("Low Flow Rate!", "danger");
      dashboard.sendUpdates();
      isSanitizing = false;
      isRising = false;
      isFlushing = false;
      isOverflowing = false;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      delay(100);
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      // digitalWrite(UVLED_PIN, HIGH);
    }
    flowCount = 0;
  }

  // Force Sanitize
  sanitize.attachCallback([&](int sanitizeValue)
                          {
    if (sanitizeValue == 0)
    {
      Serial.println("PUMP OFF");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      delay(10);
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      isForced = false;
    }
    else
    {
      digitalWrite(SANITATION_VALVE_PIN, LOW);
      digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
      delay(10);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PUMP_PWM, speed);
      isForced = true;
    }
    sanitize.update(sanitizeValue); });
  dashboard.sendUpdates();

  // Force Overflow
  overflow.attachCallback([&](int overflowValue)
                          {
    if (overflowValue == 0)
    {
      Serial.println("PUMP OFF");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      delay(10);
      digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
      isForced = false;
    }
    else
    {
      digitalWrite(OVERFLOW_VALVE_PIN, LOW);
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      delay(10);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PUMP_PWM, speed);
      isForced = true;
    }
    overflow.update(overflowValue); });
  dashboard.sendUpdates();

  pumpSpeed.attachCallback([&](int pumpSpeedValue)
                           {
    speed = pumpSpeedValue;
    pumpSpeed.update(pumpSpeedValue);
    Serial.println("PUMP SPEED CHANGED");
    Serial.println(speed); });

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}