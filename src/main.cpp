#include "CytronMotorDriver.h"
#include <Arduino.h>
#include <PID_v1.h>
#include "RPi_Pico_TimerInterrupt.h"
#include "Commands.h"
#include "BTS7960.h"
#include "Cytron_MD30C.h"
#include <FastLED.h>

// Status Led init
#define LED_PIN 28
#define NUM_LEDS 5
#define BRIGHTNESS 255
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

<<<<<<< HEAD
bool CONNECTED = false; // Stores the Drive Hub connection status
bool FIRST_BOOT = true; // Stores the Drive Hub first connection status
int warning_led_blink_rate = 200; // In Millisecond
const int time_out = 100;
// Set everything to 0 if no new data is received within the next 100 ms of receiving data

unsigned long previousMillis = 0;            // Stores the last time the LED was updated
unsigned long previousMillis_discon_led = 0; // Stores the last time the LED was updated
bool connection_led_state = false;

=======
>>>>>>> 1a0b34323716d05ef3f12f02300d50ab10701106
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
  unsigned long currentMillis = millis();
  CONNECTED = (currentMillis - previousMillis <= time_out) ? true : false;

  if (Serial.available() > 0)
  {
    byte cmd = Serial.read();
    run_commands(cmd);
    previousMillis = currentMillis;
  }

<<<<<<< HEAD
  // Power up the motor if DrveHub is connected
  if (CONNECTED){
    motor_1.setSpeed(DriveData.motor_1);
    motor_2.setSpeed(DriveData.motor_2);
    motor_3.setSpeed(DriveData.motor_3);
    motor_4.setSpeed(DriveData.motor_4);

    // TODO : turn on only once not every time 
    leds[0] = CRGB::Green;
    leds[1] = CRGB::Green;
    leds[2] = CRGB::Green;
    leds[3] = CRGB::Green;
    leds[4] = CRGB::Green;
    FastLED.show();
  }

  if (!CONNECTED){
    motor_1.setSpeed(0);
    motor_2.setSpeed(0);
    motor_3.setSpeed(0);
    motor_4.setSpeed(0);
     // Disconnection Led Warning
    if (currentMillis - previousMillis_discon_led >= warning_led_blink_rate)
    {
      connection_led_state = !connection_led_state;

      if (connection_led_state == true)
      {
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        leds[2] = CRGB::Red;
        leds[3] = CRGB::Red;
        leds[4] = CRGB::Red;
      }
      else
      {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        leds[2] = CRGB::Black;
        leds[3] = CRGB::Black;
        leds[4] = CRGB::Black;
      }
      previousMillis_discon_led = currentMillis;
    FastLED.show();
    }
  }
=======
  motor_1.setSpeed(DriveData.motor_1);
  motor_2.setSpeed(DriveData.motor_2);
  motor_3.setSpeed(DriveData.motor_3);
  motor_4.setSpeed(DriveData.motor_4);
>>>>>>> 1a0b34323716d05ef3f12f02300d50ab10701106

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
  Serial.println(DriveData.motor_4);
#endif
}