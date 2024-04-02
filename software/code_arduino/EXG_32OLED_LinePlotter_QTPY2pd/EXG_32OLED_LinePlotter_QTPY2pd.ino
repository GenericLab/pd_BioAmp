// ECG Filter - BioAmp EXG Pill
// and some remaining code from EMG Envelop - BioAmp EXG Pill
// https://github.com/upsidedownlabs/BioAmp-EXG-Pill

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2021 Upside Down Labs - contact@upsidedownlabs.tech

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*
 *  Draws a scrolling graph of a single analog input (set to A0) on the OLED display
 *
 *  By Jon E. Froehlich
 *  @jonfroehlich
 *  http://makeabilitylab.io
 */

#include <Adafruit_NeoPixel.h>
#include "logos.h"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SDA_PIN 4
#define SCL_PIN 5
int buttonPin = 6;  // the number of the pushbutton pin
#define PINK_PIN 7
#define LED_PIN 8
#define PIEZO_PIN 9
#define GATE_PIN 10 // not sure what was connected in Serv's board

#define DAC_PIN A0
#define POT_PIN A1
#define INPUT_PIN A2

#define LED_COUNT 6
#define BRIGHTNESS 10

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

#define SAMPLE_RATE 125
#define DISPLAY_RATE 25
#define BAUD_RATE 115200

int buttonState = HIGH;              // the current reading from the input pin
int lastButtonState = HIGH;          // the previous reading from the input pin
int screenState = 1;                 // different views on the OLED screen
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
int reading;

//#define BUFFER_SIZE 128 // hope to increase this... only for EMG envelop

int MIN_ANALOG_INPUT = -2000;
int MAX_ANALOG_INPUT = 2000;

const int DELAY_LOOP_MS = 1;  // change to slow down how often to read and graph value

//int circular_buffer[BUFFER_SIZE];
int data_index, sum;

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int _circularBuffer[SCREEN_WIDTH];  //fast way to store values
int _curWriteIndex = 0;             // tracks where we are in the circular buffer

int displayUpdate;
int lastValue;
//float signal;

// for tracking fps
float _fps = 0;
unsigned long _frameCount = 0;
unsigned long _fpsStartTimeStamp = 0;

// status bar
boolean _drawStatusBar = false;  // change to show/hide status bar
int _graphHeight = SCREEN_HEIGHT;


void setup() {
  analogReadResolution(12);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(GATE_PIN, OUTPUT);
  pinMode(PINK_PIN, OUTPUT);
  analogWrite(PINK_PIN, 85);
  Serial.begin(BAUD_RATE);

  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);  // Set BRIGHTNESS to about 1/5 (max = 255)
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!_display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  // Clear the buffer
  _display.clearDisplay();
  _display.setRotation(2);
  _display.setTextSize(1);
  _display.setTextColor(WHITE, BLACK);
  _display.setCursor(0, 0);
  _display.println("Screen initialized!");
  _display.display();
  delay(500);
  _display.clearDisplay();
  tone(9, 800, 20);
  //rainbow(1);  // Flowing rainbow cycle along the whole strip
    // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<LED_COUNT; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    strip.setPixelColor(i, 255, 0, 200, 0);

    strip.show();   // Send the updated pixel colors to the hardware.
    
    _display.clearDisplay();  // Make sure the display is cleared
    _display.drawBitmap(0, 0, epd_bitmap_allArray[i], 128, 32, WHITE);
    // Update the display
    _display.display();
    delay(500);

  }

    // Clear the buffer
  _display.clearDisplay();
  _display.setRotation(2);
  _display.setTextSize(1);
  _display.setTextColor(WHITE, BLACK);
  _display.setCursor(0, 0);
  _display.println("Based on BioAmp EXG");
  _display.setCursor(0, 12);
  _display.println("by Upside Down Labs");
  _display.setCursor(0, 24);
  _display.println("OSHW IN000026");
  _display.display();
  delay(1000);


    _display.clearDisplay();  // Make sure the display is cleared
    _display.drawBitmap(0, 0, epd_bitmap_oshw_IN0000026, 128, 32, WHITE);
    // Update the display
    _display.display();
    delay(1000);

    _display.clearDisplay();  // Make sure the display is cleared
    _display.drawBitmap(0, 0, grouchoFractal, 128, 32, WHITE);
    // Update the display
    _display.display();
    tone(9, 500, 400);
    delay(300);
    tone(9, 600, 300);
    delay(300);
    tone(9, 700, 200);
    delay(300);
    tone(9, 800, 1000);

    delay(1000);



  if (_drawStatusBar) {
    _graphHeight = SCREEN_HEIGHT - 16;
  }

  _fpsStartTimeStamp = millis();

  int displayUpdate = SAMPLE_RATE / DISPLAY_RATE;
}

void loop() {

  // Calculate elapsed time
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;
  // read Button

  reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;



      if (buttonState == HIGH) {
        screenState = !screenState;
        for (int i = 0; i < _display.width(); i++) {
          _circularBuffer[i] = -10000;
        }
        if (screenState == 0) {
        MIN_ANALOG_INPUT = -100;
        MAX_ANALOG_INPUT = 4095;
        _drawStatusBar = true;
        _graphHeight = SCREEN_HEIGHT - 8;
        }
        if (screenState == 1) {
        MIN_ANALOG_INPUT = -1800;
        MAX_ANALOG_INPUT = 1000;
        _drawStatusBar = true;
        _graphHeight = SCREEN_HEIGHT - 8;
        _display.clearDisplay();  // Make sure the display is cleared
        _display.drawBitmap(0, 0, epd_bitmap_exg_icons_ecg, 128, 32, WHITE);
        // Update the display
        _display.display();
            tone(9, 700, 200);
        delay(300);
        delay(300);
        }

      }
    }
  }


  // Sample and get envelop
  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    float sensor_value = analogRead(INPUT_PIN);
    int adc1 = analogRead(A1);
    float signal = ECGFilter(sensor_value);
    int serialbyte = map(signal, -2000, 2000, 0, 255);
    if (serialbyte<=0) serialbyte = 0;
    if (serialbyte>=255) serialbyte = 255;
    int pink = signal;
    if (pink < 0) {
      pink = 0;
    }
    analogWrite(PINK_PIN, pink >> 3);
    printValues(adc1, signal);

    if (screenState == 0) {
      lastValue = adc1;
      int audioOut = map(adc1, 0, 4093, 0, 1023);
      analogWrite(DAC_PIN, audioOut);
    }
    if (screenState == 1) {
      lastValue = signal;
      if (signal>=adc1-2000) 
       {digitalWrite(GATE_PIN,1);
       tone(9, 700, 200);
       }
      else {
        digitalWrite(GATE_PIN,0);
        noTone(9);
      }
      int audioOut = map(signal, -2000, 2000, 0, 1023);
      if (audioOut<=0) serialbyte = 0;
      if (audioOut>=1023) serialbyte = 1023;
      analogWrite(DAC_PIN, audioOut);
    }


    if (displayUpdate <= 0) {
      if (screenState == 0) {
        _circularBuffer[_curWriteIndex++] = adc1;

      }
      if (screenState == 1) {
        _circularBuffer[_curWriteIndex++] = signal;
      }

      displayUpdate = SAMPLE_RATE / DISPLAY_RATE;
      updateDisplay();
      for(int i=0; i<(adc1/682); i++) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // Here we're using a moderately bright green color:
      strip.setPixelColor(i, 255, 0, 200, 0);

      strip.show();   // Send the updated pixel colors to the hardware.

      }
      for(int i=6; i>(adc1/682); i--) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // Here we're using a moderately bright green color:
      strip.setPixelColor(i, 0, 0, 0, 0);

      strip.show();   // Send the updated pixel colors to the hardware.

      }
      //printValues(adc1, signal);
    } else {
      displayUpdate--;
    }
  }
  lastButtonState = reading;
}

/**
 * update the display
 */
void updateDisplay() {
  // Clear the display on each frame. We draw from the _circularBuffer
  _display.clearDisplay();
  // Set the circular buffer index back to zero when it reaches the
  // right of the screen
  if (_curWriteIndex >= _display.width()) {
    _curWriteIndex = 0;
  }

  if (_drawStatusBar) {
    drawStatusBar(lastValue);
  }

  // Draw the line graph based on data in _circularBuffer
  int xPos = 0;
  for (int i = _curWriteIndex; i < _display.width(); i++) {
    int envelop = _circularBuffer[i];
    int envelop_last = _circularBuffer[i - 1];
    drawLine(xPos, envelop, envelop_last);
    xPos++;
  }

  for (int i = 1; i < _curWriteIndex; i++) {
    int envelop = _circularBuffer[i];
    int envelop_last = _circularBuffer[i - 1];
    drawLine(xPos, envelop, envelop_last);
    xPos++;
    ;
  }

  _display.display();
  calcFrameRate();
  delay(DELAY_LOOP_MS);
}

/**
 * Draw the line at the given x position and analog value
 */
void drawLine(int xPos, int envelop, int envelop_last) {
  int lineHeight = map(envelop, MIN_ANALOG_INPUT, MAX_ANALOG_INPUT, 0, _graphHeight);
  int lineBottom = map(envelop_last, MIN_ANALOG_INPUT, MAX_ANALOG_INPUT, 0, _graphHeight);
  int yPos = _display.height() - lineHeight;
  if (screenState == 0) {
    _display.drawFastVLine(xPos, yPos - 1, abs(lineBottom - lineHeight + 1), SSD1306_WHITE);
  }
  if (screenState == 1) {
    _display.drawFastVLine(xPos, yPos, abs(lineBottom - lineHeight + 1), SSD1306_WHITE);
  }
}

void printValues(int value1, int value2) {
  //   Serial.print(reading);
  //		Serial.print("\t");
  //  Serial.print(screenState);
  //		Serial.print("\t");
  Serial.print(value1);
  Serial.print(" ");
  //Serial.write(value1);
  Serial.println(value2);
}

/**
 * Call this every frame to calculate frame rate
 */
void calcFrameRate() {

  unsigned long elapsedTime = millis() - _fpsStartTimeStamp;
  _frameCount++;
  if (elapsedTime > 1000) {
    _fps = _frameCount / (elapsedTime / 1000.0);
    _fpsStartTimeStamp = millis();
    _frameCount = 0;
  }
}

/**
 * Draws the status bar at top of screen with fps and analog value
 */
void drawStatusBar(int displayVal) {

  // erase status bar by drawing all black
  _display.fillRect(0, 0, _display.width(), 16, SSD1306_BLACK);

  // Draw frame count
  int16_t x1, y1;
  uint16_t w, h;

  if (screenState == 0) {
    /*
    _display.setTextSize(1);
    _display.getTextBounds("XX.XX fps", 0, 0, &x1, &y1, &w, &h);
    _display.setCursor(_display.width() - w, 8);
    _display.print(_fps);
    _display.print(" fps");
    */
    // Draw current val
    _display.setTextSize(1);
    _display.setCursor(0, 0);
    _display.print(displayVal);
    _display.setTextSize(1);
    _display.getTextBounds("XXXX ANALOG", 0, 0, &x1, &y1, &w, &h);
    _display.setCursor(_display.width() - w, 0);
    _display.print("adc1");
    //_display.print(INPUT_PIN);
    _display.print(" ANALOG");
  }
  if (screenState == 1) {
    _display.setTextSize(1);
    _display.setCursor(0, 0);
    _display.print("ECG");
    _display.getTextBounds("XX ECG-FILTERED", 0, 0, &x1, &y1, &w, &h);
    _display.setCursor(_display.width() - w, 0);
    //_display.print("A");
    //_display.print(INPUT_PIN);
    //_display.print(" ECG-FILTERED");
  }
}

/* Envelop detection algorithm
int getEnvelop(int abs_emg){
	sum -= circular_buffer[data_index];
	sum += abs_emg;
	circular_buffer[data_index] = abs_emg;
	data_index = (data_index + 1) % BUFFER_SIZE;
	return (sum/BUFFER_SIZE) * 2;
}
*/

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference:
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EMGFilter(float input) {
  float output = input;
  {
    static float z1, z2;  // filter section state
    float x = output - 0.05159732 * z1 - 0.36347401 * z2;
    output = 0.01856301 * x + 0.03712602 * z1 + 0.01856301 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - -0.53945795 * z1 - 0.39764934 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - 0.47319594 * z1 - 0.70744137 * z2;
    output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - -1.00211112 * z1 - 0.74520226 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 125.0 Hz, frequency: [0.5, 44.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference:
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float ECGFilter(float input) {
  float output = input;
  {
    static float z1, z2;  // filter section state
    float x = output - 0.70682283 * z1 - 0.15621030 * z2;
    output = 0.28064917 * x + 0.56129834 * z1 + 0.28064917 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - 0.95028224 * z1 - 0.54073140 * z2;
    output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - -1.95360385 * z1 - 0.95423412 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - -1.98048558 * z1 - 0.98111344 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 75.0 Hz, frequency: [0.5, 19.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference:
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EOGFilter(float input) {
  float output = input;
  {
    static float z1, z2;  // filter section state
    float x = output - 0.02977423 * z1 - 0.04296318 * z2;
    output = 0.09797471 * x + 0.19594942 * z1 + 0.09797471 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - 0.08383952 * z1 - 0.46067709 * z2;
    output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - -1.92167271 * z1 - 0.92347975 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;  // filter section state
    float x = output - -1.96758891 * z1 - 0.96933514 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

void showLogo_hackteria() {
  _display.clearDisplay();  // Make sure the display is cleared
  _display.drawBitmap(0, 0, hackteria_logo, 128, 32, WHITE);
  _display.setTextSize(2);
  _display.setTextColor(WHITE);
  _display.setCursor(11, 49);
  _display.println("HACKTERIA");
  // Update the display
  _display.display();
  delay(30);
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show();  // Update strip with new contents
    delay(wait);   // Pause for a moment
  }
}
