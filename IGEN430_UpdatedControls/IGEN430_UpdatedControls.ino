#include <NewPing.h>

// Constants
#define FLOW_PIN 2
#define OVERFLOW_VALVE_PIN 3
#define SANITATION_VALVE_PIN 4
// #define UVLED_PIN 5
#define PUMP_PWM 11
#define IN1 12
#define IN2 13
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define minFlowRate 0.2 // L/min
#define levelMax 5
#define levelMid 15
#define levelMin 30 // 30 is best
#define SPEED 200

// Variables
float waterLevel;
float avgWaterLevel;
float prevLevel;
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
unsigned int uS;
volatile int flowCount = 0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200);

void setup()
{
    Serial.begin(9600);
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
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(SANITATION_VALVE_PIN, HIGH);
    digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
    // digitalWrite(UVLED_PIN, HIGH);
    uS = sonar.ping_median(5);            // Send ping, get ping time in microseconds (uS).
    avgWaterLevel = sonar.convert_cm(uS); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    prevLevel = avgWaterLevel;
}

void loop()
{
    if ((millis() - levelCheckTime) > 1000 || millis() < 1000)
    {
        // Check water level
        // digitalWrite(TRIGGER_PIN, LOW);
        // delayMicroseconds(2);
        // digitalWrite(TRIGGER_PIN, HIGH);
        // delayMicroseconds(10);
        // digitalWrite(TRIGGER_PIN, LOW);
        // float duration = pulseIn(ECHO_PIN, HIGH);
        // waterLevel = duration * 0.034 / 2; // cm
        // avgWaterLevel = getAverageWaterLevel(5);
        uS = sonar.ping_median(5);            // Send ping, get ping time in microseconds (uS).
        avgWaterLevel = sonar.convert_cm(uS); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
        Serial.print("Water level: ");
        Serial.println(avgWaterLevel);
        Serial.print("Prev Level: ");
        Serial.println(prevLevel);
        // float avgWaterLevel = waterLevel;
        levelCheckTime = millis();
    }

    if (avgWaterLevel < levelMin && avgWaterLevel > levelMax && !isFlushing && !isSanitizing && !isOverflowing)
    {
        // Serial.println("Waiting Mode");
        digitalWrite(SANITATION_VALVE_PIN, HIGH);
        digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
        // digitalWrite(UVLED_PIN, HIGH);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }

    // Check if water level is rising or falling
    bool isCurrentlyRising = avgWaterLevel < prevLevel;
    if (isCurrentlyRising != isRising && !isRising && millis() > 5000) //&& (millis() - flushIntervalTime) > flushInterval)
    {
        // Water level has changed direction
        Serial.println("Flush beginning, opening overflow valve for 30 minutes");
        isRising = isCurrentlyRising;
        isFlushing = true;
        digitalWrite(OVERFLOW_VALVE_PIN, LOW);
        flushStartTime = millis();
        flushIntervalTime = millis();
    }

    // Check if overflow valve needs to be opened
    if (avgWaterLevel < levelMax && !isSanitizing && millis() > 5000)
    {
        if (!isOverflowing)
        {
            Serial.println("Overflow valve opened");
            isOverflowing = true;
            digitalWrite(OVERFLOW_VALVE_PIN, LOW);
            delay(10);
            isRising = false;
        }
    }
    else if (avgWaterLevel > levelMin && isOverflowing)
    {
        Serial.println("Overflow valve closed");
        isOverflowing = false;
        delay(10);
        digitalWrite(OVERFLOW_VALVE_PIN, HIGH);
    }

    // Check if sanitation valve needs to be opened
    if ((millis() - flushStartTime) > flushDuration && isFlushing)
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
            analogWrite(PUMP_PWM, SPEED);
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
    if (millis() - lastFlowCheckTime >= 1000 && isSanitizing)
    {
        lastFlowCheckTime = millis();
        float flowRate = (((float)flowCount) / 23);
        Serial.print("Flow rate: ");
        Serial.print(flowRate);
        Serial.println(" L/min");
        if (flowRate < minFlowRate && isSanitizing)
        {
            Serial.println("Flow rate too low, closing sanitation valve and turning off pump and UV LED");
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
}

void flowISR()
{
    flowCount++;
}

float getAverageWaterLevel()
{
    float readings[10];
    static int index = 0;
    float total = 0;
    readings[index] = avgWaterLevel;
    index = (index + 1) % 10;
    for (int i = 0; i < 10; i++)
    {
        total += readings[i];
    }
    return total / 10;
}
