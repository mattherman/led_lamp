#include "Arduino.h"
#include "Adafruit_NeoPixel.h"

/*
 * References / Examples
 * 
 * ESP32-DevKitC V4 Getting Started Guide (and pin layout)
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html
 * 
 * NeoPixel Library
 * https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use
 * 
 * Advanced IoT Cloud Stuff...
 * 
 * Connecting ESP32 & ESP8266 to Arduino Cloud IoT
 * https://docs.arduino.cc/cloud/iot-cloud/tutorials/esp-32-cloud
 * 
 * Building an AWS IoT Core device using AWS Serverless and an ESP32
 * https://aws.amazon.com/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/
 * 
 */

#define TOUCH_PIN 4
#define TOUCH_MIN 15

#define LED_PIN 0
#define LED_COUNT 24

#define MIN_RAINBOW_MS 10000
#define RAINBOW_WAIT 20

#define MAX_BRIGHTNESS 255
#define MAX_SATURATION 255
#define MAX_HUE 65536
#define HUE_STEP 255

Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// this function gets called once to initialize stuff
void setup()
{
  Serial.begin(115200);
  Serial.println("start");

  ring.begin();
  ring.clear();
  ring.show();
}

// this function gets called periodically
void loop()
{
  uint16_t touch_value = touchRead(TOUCH_PIN);
  if (touch_value < TOUCH_MIN)
  {
    rainbow(RAINBOW_WAIT);
  }
}

void rainbow(int wait) {
  uint32_t pixel_hue_offset = 0;
  uint8_t brightness_value = 0;

  // ramp up
  while (++brightness_value < MAX_BRIGHTNESS) {
    rainbow_cycle(wait, pixel_hue_offset, brightness_value);
  }

  uint32_t timestamp = millis();
  while (millis() - timestamp < MIN_RAINBOW_MS) {
    rainbow_cycle(wait, pixel_hue_offset, brightness_value);
  }

  // ramp down
  while (--brightness_value > 0) {
    rainbow_cycle(wait, pixel_hue_offset, brightness_value);
  }

  ring.clear();
  ring.show();
}

void rainbow_cycle(int wait, uint32_t& pixel_hue_offset, uint8_t brightness) {
    for (int i = 0; i < LED_COUNT; ++i) {
      uint16_t pixel_hue_value = pixel_hue_offset + (i * MAX_HUE / LED_COUNT);
      ring.setPixelColor(i, ring.gamma32(ring.ColorHSV(pixel_hue_value, MAX_SATURATION, brightness)));
    }

    ring.show();
    delay(wait);  

    pixel_hue_offset += HUE_STEP;
}
