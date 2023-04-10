// Required Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <Smoothed.h>

// Pin out - Updated for ESP32
#define LED 2
#define FLOW_PIN 21             // GPIO 21
#define OVERFLOW_VALVE_PIN 23   // GPIO 23
#define SANITATION_VALVE_PIN 22 // GPIO 22
#define UVLED_PIN 16            // GPIO 4
#define PUMP_PWM 19             // GPIO 19
#define IN1 18                  // GPIO 18
#define IN2 5                   // GPIO 5
#define TRIGGER_PIN 4           // GPIO 17
#define ECHO_PIN 15             // GPIO 16

// Constants
#define minFlowRate 1.9 // L/min
#define levelMax 5
#define levelMid 15
#define levelMin 30 // 30 is best

// Variables
int speed = 0;
int flag = 0;
float waterLevel = 0;
float avgWaterLevel = 0;
int prevLevel = 0;
int percentFullSanitized = 0;
int percentFullRaw = 0;
bool isForced = false;
bool isOverflowing = false;
bool isSanitizing = false;
bool isRising = false;
bool isFlushing = false;
bool forceSanitize = false;
unsigned long flushInterval = 5 * 60 * 1000; // 60 * 1000; // 5 hours
unsigned long flushDuration = 30 * 1000;     //* 60 * 1000;     // 30 minutes
unsigned long flushStartTime = 0;
unsigned long lastFlowCheckTime = 0;
unsigned long levelCheckTime = 0;
unsigned long flushIntervalTime = 0;
unsigned long flushTimer = 0;
unsigned long prevLevelCheckTime = 0;
unsigned long dashboardUpdateTime = 0;
unsigned int uS;
volatile byte flowCount = 0;
byte flowCount1sec = 0;
float calibrationFactor = 13; // Flow rate sensor calibration factor
float flowRate = 0;
float flowLitres = 0;
float totalLitres = 0;

// Wifi Credentials
const char *ssid = "BTW - Pro Max 13";
const char *password = "14881488";

// Dashboard and Server Setup
AsyncWebServer server(80);
ESPDash dashboard(&server);

// Dashboard Cards
Card waterHeigth1(&dashboard, GENERIC_CARD, "Raw Water Tank", "%");
Card volume(&dashboard, GENERIC_CARD, "Sanitized Water", "L");
Card flow(&dashboard, GENERIC_CARD, "Flow Rate", "L/min");
Card systemStatus(&dashboard, STATUS_CARD, "System Status", "idle");
Card overflow(&dashboard, BUTTON_CARD, "Force Overflow");
Card sanitize(&dashboard, BUTTON_CARD, "Force Sanitize");
Card reset(&dashboard, BUTTON_CARD, "Reset Water Level");
Card pumpSpeed(&dashboard, SLIDER_CARD, "Pump Speed", "", 0, 255);
Card calibration(&dashboard, SLIDER_CARD, "Flow Calibration", "", 0, 36);
Chart waterHeightChart(&dashboard, BAR_CHART, "Raw Water Tank %");

String XAxis[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"};
int YAxis[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Ultrasonic Sensor
NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200);

// Smoothing Value
Smoothed<float> smooth;

void IRAM_ATTR flowISR()
{
  flowCount++;
}

int percentWaterLevel(float level)
{
  return ((levelMin - level) / (levelMin - levelMax) * 100);
}

void setup()
{
  // Serial Communication Setup
  Serial.begin(115200);

  // Pins Setup
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, FALLING);
  pinMode(SANITATION_VALVE_PIN, OUTPUT);
  pinMode(OVERFLOW_VALVE_PIN, OUTPUT);
  pinMode(UVLED_PIN, OUTPUT);
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
  digitalWrite(UVLED_PIN, HIGH);

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

  smooth.begin(SMOOTHED_AVERAGE, 10);

  for (int k = 0; k <= 5; k++)
  {
    uS = sonar.ping_median(5);         // Send ping, get ping time in microseconds (uS).
    waterLevel = sonar.convert_cm(uS); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    smooth.add(waterLevel);
    avgWaterLevel = smooth.get();
    prevLevel = avgWaterLevel;
  }
}

void loop()
{
  if ((millis() - levelCheckTime) > 1000)
  {
    uS = sonar.ping_median(5);         // Send ping, get ping time in microseconds (uS).
    waterLevel = sonar.convert_cm(uS); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    smooth.add(waterLevel);
    avgWaterLevel = smooth.get();
    if (millis() - prevLevelCheckTime > 5 * 60 * 1000)
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
    if (millis() - lastFlowCheckTime > 1000 && forceSanitize == true)
    {
      flowCount1sec = flowCount;
      flowCount = 0;
      flowRate = ((1000.0 / (millis() - lastFlowCheckTime)) * flowCount1sec) / calibrationFactor;
      lastFlowCheckTime = millis();
      Serial.print("Flow rate: ");
      Serial.print(flowRate);
      Serial.println(" L/min");
      flowLitres = flowRate / 60;
      totalLitres += flowLitres;
      Serial.print("Total litres: ");
      Serial.println(totalLitres);
      flow.update(flowRate);
      volume.update(totalLitres);
      dashboard.sendUpdates();
    }
  }

  // Update Dashboard
  if ((millis() - dashboardUpdateTime) > 1 * 60 * 1000)
  {
    // Insert the newest avgWaterLevel into the front of the array and shift the old values back
    percentFullRaw = percentWaterLevel(avgWaterLevel);
    for (int i = 10; i > 0; i--)
    {
      YAxis[i] = YAxis[i - 1];
    }
    YAxis[0] = percentFullRaw;
    waterHeigth1.update(percentFullRaw);
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
    digitalWrite(UVLED_PIN, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    flow.update(0);
    systemStatus.update("Idle", "idle");
    dashboard.sendUpdates();
  }

  // Check if water level is rising or falling
  bool isCurrentlyRising = avgWaterLevel - 0.5 < prevLevel;
  if (isCurrentlyRising != isRising && !isRising && millis() > 5000 && !isForced && millis() - flushTimer > flushInterval)
  {
    // Water level has changed direction
    Serial.println("Flush beginning");
    isRising = isCurrentlyRising;
    isFlushing = true;
    digitalWrite(SANITATION_VALVE_PIN, HIGH);
    digitalWrite(OVERFLOW_VALVE_PIN, LOW);
    delay(10);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PUMP_PWM, speed);
    flushStartTime = millis();
    flushIntervalTime = millis();
    flushTimer = millis();
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
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      digitalWrite(OVERFLOW_VALVE_PIN, LOW);
      delay(10);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PUMP_PWM, speed);
      isRising = false;
      systemStatus.update("Overflow", "danger");
      dashboard.sendUpdates();
    }
  }
  else if (avgWaterLevel > levelMid && isOverflowing && !isForced)
  {
    Serial.println("Overflow valve closed");
    isOverflowing = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
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
      digitalWrite(UVLED_PIN, LOW);
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
      digitalWrite(UVLED_PIN, HIGH);
    }
  }
  // Check flow rate
  if (millis() - lastFlowCheckTime > 1000 && isSanitizing && !isForced)
  {
    flowCount1sec = flowCount;
    flowCount = 0;
    flowRate = ((1000.0 / (millis() - lastFlowCheckTime)) * flowCount1sec) / calibrationFactor;
    lastFlowCheckTime = millis();
    Serial.print("Flow rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");
    flowLitres = flowRate / 60;
    totalLitres += flowLitres;
    Serial.print("Total litres: ");
    Serial.println(totalLitres);
    flow.update(flowRate);
    volume.update(totalLitres);
    dashboard.sendUpdates();
    flag += 1;
    if (flowRate < minFlowRate && isSanitizing && flag == 5)
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
      delay(10);
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      delay(10);
      digitalWrite(UVLED_PIN, HIGH);
      flag = 0;
      delay(1000);
    }
  }

  // Force Sanitize
  sanitize.attachCallback([&](int sanitizeValue)
                          {
    if (sanitizeValue == 0)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      delay(10);
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
      delay(10);
      digitalWrite(UVLED_PIN, HIGH);
      isForced = false;
      forceSanitize = false;
    }
    else
    {
      if (avgWaterLevel <= levelMin){
        digitalWrite(UVLED_PIN, LOW);
        delay(10);
        digitalWrite(SANITATION_VALVE_PIN, LOW);
        digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
        delay(10);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(PUMP_PWM, speed);
        isForced = true;
        forceSanitize = true;
      }
      else{
        systemStatus.update("Low water level!", "danger");
        dashboard.sendUpdates();
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        delay(10);
        digitalWrite(SANITATION_VALVE_PIN, HIGH);
        delay(10);
        digitalWrite(UVLED_PIN, HIGH);
        delay(5000);
        isForced = false;
        forceSanitize = false;
      }

    }
    sanitize.update(sanitizeValue); });
  dashboard.sendUpdates();

  // Force Overflow
  overflow.attachCallback([&](int overflowValue)
                          {
    if (overflowValue == 0)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      delay(10);
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
      isForced = false;
    }
    else
    {
      if (avgWaterLevel <= levelMin){
      digitalWrite(OVERFLOW_VALVE_PIN, LOW);
      digitalWrite(SANITATION_VALVE_PIN, HIGH);
      delay(10);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PUMP_PWM, speed);
      isForced = true;
      }
      else{
        systemStatus.update("Low water level!", "danger");
        dashboard.sendUpdates();
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        delay(10);
        digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
        delay(5000);
        isForced = false;
      }
    }
    overflow.update(overflowValue); });
  dashboard.sendUpdates();

  pumpSpeed.attachCallback([&](int pumpSpeedValue)
                           {
    speed = pumpSpeedValue;
    pumpSpeed.update(pumpSpeedValue); });

  calibration.attachCallback([&](float calibrationValue = 13)
                             {
    calibrationFactor = calibrationValue;
    calibration.update(calibrationValue); });

  reset.attachCallback([&](int resetButton)
                       {
    if (resetButton == 0)
    {
      volume.update(totalLitres);
      dashboard.sendUpdates();
    }
    else
    {
      totalLitres = 0;
      volume.update(totalLitres);
      dashboard.sendUpdates();
    }
    reset.update(resetButton); });

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}