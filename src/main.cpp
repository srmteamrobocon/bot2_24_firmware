#include "CytronMotorDriver.h"
#include <Arduino.h>
#include <PID_v1.h>
#include "RPi_Pico_TimerInterrupt.h"
#include "Commands.h"
#include "BTS7960.h"
#include "Cytron_MD30C.h"
#include <FastLED.h>

// Status Led init
#define LED_PIN 17
#define NUM_LEDS 2
#define BRIGHTNESS 200
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];


#define _LOG_MOTOR_PWM_ 0
#define _LOG_ENCODER_TICK_RPM_ 0
#define _LOG_PID_STATS_ 0

// Specify motor Driver
#define _MDDS30_
// #define _Motor_cytron_md30_
struct BusData
{
  int16_t motor_1;
  int16_t motor_2;
  int16_t motor_3;
  int16_t motor_4;
  int8_t led_status;
};
struct BusData DriveData = {0, 0, 0, 0};

#ifdef _MDDS30_
CytronMD motor_1(PWM_DIR, 8, 12);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor_2(PWM_DIR, 11, 13); // PWM 2 = Pin 9, DIR 2 = Pin 10.
CytronMD motor_3(PWM_DIR, 22, 20); // PWM 2 = Pin 9, DIR 2 = Pin 10.
CytronMD motor_4(PWM_DIR, 26, 21); // PWM 2 = Pin 9, DIR 2 = Pin 10.
#endif

void run_commands(char COMMAND)
{
  switch (COMMAND)
  {
  case MOTOR_RAW_PWM:
    // Adding braces removes jump case error
    {
      char buffer_motor_1_PWM[5];
      char buffer_motor_2_PWM[5];
      char buffer_motor_3_PWM[5];
      char buffer_motor_4_PWM[5];

      Serial.readBytesUntil(',', buffer_motor_1_PWM, sizeof(buffer_motor_1_PWM));
      Serial.readBytesUntil(',', buffer_motor_2_PWM, sizeof(buffer_motor_2_PWM));
      Serial.readBytesUntil(',', buffer_motor_3_PWM, sizeof(buffer_motor_3_PWM));
      Serial.readBytesUntil(',', buffer_motor_4_PWM, sizeof(buffer_motor_4_PWM));

      // Not sure about its use case
      char endBiyte = Serial.read();

      // Convert valid PWM char array to Intiger and store it to Struct
      DriveData.motor_1 = atoi(buffer_motor_1_PWM);
      DriveData.motor_2 = atoi(buffer_motor_2_PWM);
      DriveData.motor_3 = atoi(buffer_motor_3_PWM);
      DriveData.motor_4 = atoi(buffer_motor_4_PWM);

      break;
    }

  case READ_ENCODERS:
  {
    // Serial.print(" ");
    // Serial.print(counts);
    // Serial.print(counts);
    // Serial.print(" ");
    // Serial.println(counts);
    break;
  }
  }
}

void setup()
{
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  Serial.setTimeout(1000);
  Serial.begin(115200);

  motor_1.setSpeed(0);
  motor_2.setSpeed(0);
  motor_3.setSpeed(0);
  motor_4.setSpeed(0);

  delay(1000);

  // Status led init
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  
}

// CORE 1 Main Loop
void loop()
{
  if (Serial.available() > 0)
  {
    byte cmd = Serial.read();
    run_commands(cmd);
  }


  motor_1.setSpeed(DriveData.motor_1);
  motor_2.setSpeed(DriveData.motor_2);
  motor_3.setSpeed(DriveData.motor_3);
  motor_4.setSpeed(DriveData.motor_4);

  // Serial.print(DriveData.motor_1);
  // Serial.print(" ");
  // Serial.print(DriveData.motor_2);
  // Serial.print(" ");
  // Serial.print(DriveData.motor_3);
  // Serial.print(" ");
  // Serial.println(DriveData.motor_4);

#if (_LOG_MOTOR_PWM_)
  Serial.println(DriveData.motor_1);
  Serial.println(DriveData.motor_2);
  Serial.println(DriveData.motor_3);
#endif
}