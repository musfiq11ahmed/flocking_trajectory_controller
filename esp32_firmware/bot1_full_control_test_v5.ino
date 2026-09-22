#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include "secrets.h"


// 1. GLOBAL PINS & HARDWARE CONSTANTS

const int leftForward = 4, leftReverse = 5;
const int rightForward = 6, rightReverse = 7;
const int leftEncoderA = 1, leftEncoderB = 2;
const int rightEncoderA = 9, rightEncoderB = 10;

const float wheelCircumference = PI * 0.044; // 44mm wheels
const float distancePerTick = wheelCircumference / 420.0; // 420 PPR


// 2. VOLATILE ENCODER COUNTERS & ISRs

volatile long leftTicks = 0;
volatile long rightTicks = 0;

void IRAM_ATTR leftEncoderISR() {
  static unsigned long lastLeftTime = 0;
  unsigned long currentTime = micros();
  
  if (currentTime - lastLeftTime > 150) {
    // Fixed deprecated volatile warning by using += 1 instead of ++
    if (digitalRead(leftEncoderA) == digitalRead(leftEncoderB)) leftTicks += 1;
    else leftTicks -= 1;
    lastLeftTime = currentTime;
  }
}

void IRAM_ATTR rightEncoderISR() {
  static unsigned long lastRightTime = 0;
  unsigned long currentTime = micros();
  
  if (currentTime - lastRightTime > 150) {
    if (digitalRead(rightEncoderA) == digitalRead(rightEncoderB)) rightTicks -= 1;
    else rightTicks += 1;
    lastRightTime = currentTime;
  }
}


// 3. SHARED MEMORY & FREERTOS HANDLES

SemaphoreHandle_t targetMutex; 
float sharedTargetLeftMPS = 0.0;
float sharedTargetRightMPS = 0.0;

// Shared PID variables
float sharedKp = 100.0;
float sharedKi = 30.0;
float sharedKd = 0.0;

TaskHandle_t NetworkTaskHandle;
TaskHandle_t MotorTaskHandle;


// 4. TASK 1: THE NETWORK NODE (CORE 0)

void NetworkTask(void *pvParameters) {
  WiFiMulti wifiMulti;
  WiFiUDP udp;
  const int udpPort = 4210;
  char packetBuffer[255];

  Serial.println("Core 0: Connecting to Wi-Fi...");
  wifiMulti.addAP(WIFI_SSID_1, WIFI_PASS_1);
  wifiMulti.addAP(WIFI_SSID_2, WIFI_PASS_2);

  while (wifiMulti.run() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  Serial.print("\n--- WI-FI CONNECTED ---\nIP: ");
  Serial.println(WiFi.localIP());
  udp.begin(udpPort);

  for (;;) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(packetBuffer, 255);
      if (len > 0) packetBuffer[len] = '\0';

      if (strncmp(packetBuffer, "TARGET", 6) == 0) {
        float tempLeft = 0.0, tempRight = 0.0;
        sscanf(packetBuffer, "TARGET,%f,%f", &tempLeft, &tempRight);

        if (xSemaphoreTake(targetMutex, portMAX_DELAY)) {
          sharedTargetLeftMPS = tempLeft;
          sharedTargetRightMPS = tempRight;
          xSemaphoreGive(targetMutex);
        }
      }
      else if (strncmp(packetBuffer, "TUNE", 4) == 0) {
        float tempKp = 0.0, tempKi = 0.0, tempKd = 0.0;
        sscanf(packetBuffer, "TUNE,%f,%f,%f", &tempKp, &tempKi, &tempKd);
        
        if (xSemaphoreTake(targetMutex, portMAX_DELAY)) {
          sharedKp = tempKp;
          sharedKi = tempKi;
          sharedKd = tempKd;
          xSemaphoreGive(targetMutex);
        }
        Serial.printf("Live PID Updated -> Kp:%.3f Ki:%.3f Kd:%.3f\n", tempKp, tempKi, tempKd);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}


// 5. TASK 2: THE MOTOR CONTROLLER (CORE 1)

void MotorTask(void *pvParameters) {
  long prevLeftTicks = 0, prevRightTicks = 0;
  float leftIntegral = 0, rightIntegral = 0;
  float prevLeftError = 0, prevRightError = 0;
  
  float leftPWM = 150, rightPWM = 150; 
  float localTargetLeft = 0.0, localTargetRight = 0.0;
  
  // Initialize safe defaults OUTSIDE the loop
  float localKp = 100.0, localKi = 30.0, localKd = 0.0;

  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // 1. Safely grab the latest targets (No garbage memory if mutex fails)
    if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(5))) {
      localTargetLeft = sharedTargetLeftMPS;
      localTargetRight = sharedTargetRightMPS;
      localKp = sharedKp;
      localKi = sharedKi;
      localKd = sharedKd;
      xSemaphoreGive(targetMutex);
    }

    // 2. Read Encoders
    noInterrupts();
    long currentLeftTicks = leftTicks;
    long currentRightTicks = rightTicks;
    interrupts();

    // 3. Kinematics (Ticks to m/s)
    long leftVelocityTicks = currentLeftTicks - prevLeftTicks;
    long rightVelocityTicks = currentRightTicks - prevRightTicks;
    float currentLeftMPS = leftVelocityTicks * distancePerTick * 20.0;
    float currentRightMPS = rightVelocityTicks * distancePerTick * 20.0;

    // 4. PID Math 
    float leftError = localTargetLeft - currentLeftMPS;
    float rightError = localTargetRight - currentRightMPS;

    leftIntegral += leftError;
    rightIntegral += rightError;

    // Integral clamp
    const float INTEGRAL_LIMIT = 50.0;
    leftIntegral = constrain(leftIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    rightIntegral = constrain(rightIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    // Reset integral when target is zero
    if (localTargetLeft == 0.0) {
        leftIntegral = 0.0;
        leftPWM = 150;
    }
    if (localTargetRight == 0.0) {
        rightIntegral = 0.0;
        rightPWM = 150;
    }

    float leftDerivative = leftError - prevLeftError;
    float rightDerivative = rightError - prevRightError;

    float Kf = 220.0; 
    float leftFF = localTargetLeft * Kf;
    float rightFF = localTargetRight * Kf;

    leftPWM = leftFF + (localKp * leftError) + (localKi * leftIntegral) + (localKd * leftDerivative);
    rightPWM = rightFF + (localKp * rightError) + (localKi * rightIntegral) + (localKd * rightDerivative);

    
    // 5. BIDIRECTIONAL MOTOR CONSTRAINTS
    
    int l_pwm_out = 0;
    int r_pwm_out = 0;

    // --- LEFT WHEEL ---
    if (localTargetLeft == 0.0) {
      analogWrite(leftForward, 0);
      analogWrite(leftReverse, 0);
    } else {
      l_pwm_out = abs((int)leftPWM);
      if (l_pwm_out > 255) l_pwm_out = 255;
      if (l_pwm_out < 30) l_pwm_out = 30; // Minimum torque

      if (leftPWM > 0) {
        analogWrite(leftForward, l_pwm_out); analogWrite(leftReverse, 0);
      } else {
        analogWrite(leftForward, 0); analogWrite(leftReverse, l_pwm_out);
      }
    }

    // --- RIGHT WHEEL ---
    if (localTargetRight == 0.0) {
      analogWrite(rightForward, 0);
      analogWrite(rightReverse, 0);
    } else {
      r_pwm_out = abs((int)rightPWM);
      if (r_pwm_out > 255) r_pwm_out = 255;
      if (r_pwm_out < 30) r_pwm_out = 30;

      if (rightPWM > 0) {
        analogWrite(rightForward, r_pwm_out); analogWrite(rightReverse, 0);
      } else {
        analogWrite(rightForward, 0); analogWrite(rightReverse, r_pwm_out);
      }
    }

    // 6. Save states
    prevLeftTicks = currentLeftTicks;
    prevRightTicks = currentRightTicks;
    prevLeftError = leftError;
    prevRightError = rightError;

    
    // DEBUG HEARTBEAT: Exposing the internal speedometer
    static int heartbeat = 0;
    heartbeat++;
    if (heartbeat >= 20) {
      Serial.printf("Target: %.2f | Perceived Speed: L:%.2f R:%.2f m/s | L PWM: %.0f | R PWM: %.0f\n", 
                    localTargetLeft, currentLeftMPS, currentRightMPS, leftPWM, rightPWM);
      heartbeat = 0;
    }
    

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// 6. MAIN SETUP (THE DISPATCHER)

void setup() {
  Serial.begin(115200);

  // Initialize Pins
  pinMode(leftForward, OUTPUT); pinMode(leftReverse, OUTPUT);
  pinMode(rightForward, OUTPUT); pinMode(rightReverse, OUTPUT);
  pinMode(leftEncoderA, INPUT_PULLUP); pinMode(leftEncoderB, INPUT_PULLUP);
  pinMode(rightEncoderA, INPUT_PULLUP); pinMode(rightEncoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), rightEncoderISR, CHANGE);

  targetMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(NetworkTask, "NetworkTask", 4096, NULL, 1, &NetworkTaskHandle, 0);
  xTaskCreatePinnedToCore(MotorTask, "MotorTask", 4096, NULL, 2, &MotorTaskHandle, 1);
}

void loop() {
  vTaskDelete(NULL);
}
