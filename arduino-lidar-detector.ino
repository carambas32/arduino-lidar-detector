#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "LowPower.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// #define DEBUG_SERIAL
#define OLED_I2C_ADDR 0x3C
#define MAX_MEASURE_BEFORE_SLEEP 3

const byte VL53LOX_InterruptPin = 2;
const byte VL53LOX_ShutdownPin = 8;
volatile byte VL53LOX_State = HIGH;
volatile bool interrupt = false;
volatile unsigned long lastInterrupt;
byte noInterruptCount = 0;
0
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

SSD1306AsciiWire oled;

void setup() {
#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }
  Serial.println(F("Start Serial"));
#endif

  pinMode(VL53LOX_ShutdownPin, INPUT_PULLUP);
  pinMode(VL53LOX_InterruptPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#ifdef DEBUG_SERIAL
  Serial.println(F("Coucou"));
#endif

  oled.begin(&Adafruit128x32, OLED_I2C_ADDR, -1);
  oled.setContrast(0);
  oled.setFont(fixednums15x31);
  oled.print("00");

  // if lox.begin failes its becasue it was a warm boot and the VL53LOX is in
  // continues mesurement mode we can use an IO pin to reset the device in case
  // we get stuck in this mode
  while (!lox.begin()) {
#ifdef DEBUG_SERIAL
    Serial.println(F("VL53L0X KO... Reset"));
#endif
    oled.print("-");
    digitalWrite(VL53LOX_ShutdownPin, LOW);
    delay(100);
    digitalWrite(VL53LOX_ShutdownPin, HIGH);
    delay(1000);
  }

  Serial.println(F("VL53L0X started"));

  oled.println("11");
  delay(500);
  oledSleep();

  // Serial.println(F("Set GPIO Config so if range is lower the LowThreshold trigger Gpio Pin "));
  lox.setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                    VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
                    VL53L0X_INTERRUPTPOLARITY_LOW);

  // Set Interrupt Treashholds
  // Low reading set to 50mm  High Set to 100mm
  FixPoint1616_t LowThreashHold = (1200 * 65536.0);
  FixPoint1616_t HighThreashHold = (500 * 65536.0);
  lox.setInterruptThresholds(LowThreashHold, HighThreashHold, false);
  lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);

  attachInterrupt(digitalPinToInterrupt(VL53LOX_InterruptPin), wake, CHANGE);
}

void oledSleep() {
  oled.ssd1306WriteCmd(0xAE);
  oled.ssd1306WriteCmd(0);
}

void oledWake() {
  oled.ssd1306WriteCmd(0xAF);
  oled.ssd1306WriteCmd(1);
}

void wake() {
  interrupt = true;
  VL53LOX_State = digitalRead(VL53LOX_InterruptPin);
}

void setContinuousRangeTiming(unsigned int time) {
  lox.stopRangeContinuous();
  lox.startRangeContinuous(1000);
#ifdef DEBUG_SERIAL
  Serial.print(F("One measure each "));
  Serial.print(time);
  Serial.println(F("ms while MCU sleeps"));
  delay(50);
#endif
}

void loop() {
  if (noInterruptCount >= MAX_MEASURE_BEFORE_SLEEP) {
    oledSleep();
  }

  attachInterrupt(digitalPinToInterrupt(VL53LOX_InterruptPin), wake, LOW);
  setContinuousRangeTiming(noInterruptCount < MAX_MEASURE_BEFORE_SLEEP ? 50 : 1000);

  LowPower.powerDown(noInterruptCount < MAX_MEASURE_BEFORE_SLEEP ? SLEEP_1S : SLEEP_FOREVER, ADC_OFF, BOD_OFF);  // sleep until interrupt

  detachInterrupt(digitalPinToInterrupt(VL53LOX_InterruptPin));                                                  // remove interrupt

  if (interrupt && VL53LOX_State == LOW) {
    noInterruptCount = 0;
    interrupt = false;
#ifdef DEBUG_SERIAL
    Serial.println(F("Interrupt !"));
#endif
    readAndDisplayLoxMeasure();
    lox.clearInterruptMask(false);

  } else {
    if (noInterruptCount < MAX_MEASURE_BEFORE_SLEEP) {
      noInterruptCount++;
    }
  }
}

void readAndDisplayLoxMeasure() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.getRangingMeasurement(&measure, false);  // pass in 'true' to get debug data printout!

  // Serial.println(measure.RangeStatus);
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    oledWake();
    oled.setCursor(0, 0);
    oled.print(measure.RangeMilliMeter);
    oled.print("  ");

#ifdef DEBUG_SERIAL
    Serial.print(measure.RangeMilliMeter);
    Serial.println(F("mm"));
#endif
  } else {
#ifdef DEBUG_SERIAL
    Serial.println(F("Error reading sensor"));
#endif
  }
}
