#include <Wire.h>
#include "Adafruit_I2CDevice.h" 
#include "Adafruit_VL6180X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 6
#define SHT_LOX3 5

// Optional define GPIO pins to check to see if complete
//#define GPIO_LOX1 4
//#define GPIO_LOX2 3
//#define GPIO_LOX3 2

#define WINDOW_SIZE 20

// objects for the VL6180X
Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

Adafruit_VL6180X *sensors[] = {&lox1, &lox2, &lox3};

const uint8_t COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

uint8_t sensor_idx = 0;
uint8_t sensor_range_idxs[COUNT_SENSORS];
uint8_t sensor_range_samples[COUNT_SENSORS][WINDOW_SIZE];
uint32_t sensor_range_sums[COUNT_SENSORS];
uint8_t sensor_status[COUNT_SENSORS];

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if (!lox1.begin()) {
    Serial.println(F("Failed to boot first VL6180X"));
    while (1);
  }
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin()) {
    Serial.println(F("Failed to boot second VL6180X"));
    while (1);
  }
  lox2.setAddress(LOX2_ADDRESS);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if (!lox3.begin()) {
    Serial.println(F("Failed to boot third VL6180X"));
    while (1);
  }
  lox3.setAddress(LOX3_ADDRESS);
}

void round_robin_read_sensors() {
  uint8_t range_lox = sensors[sensor_idx]->readRange();
  uint8_t status_lox = sensors[sensor_idx]->readRangeStatus();

  sensor_status[sensor_idx] = status_lox;
  if (status_lox == VL6180X_ERROR_NONE) {
    uint8_t range_idx = sensor_range_idxs[sensor_idx];

    sensor_range_sums[sensor_idx] -= sensor_range_samples[sensor_idx][range_idx]; // Remove the oldest entry from the sum
    sensor_range_samples[sensor_idx][range_idx] = range_lox; // Add the newest reading to the window

    sensor_range_sums[sensor_idx] += range_lox; // Add the newest reading to the sum
    
    sensor_range_idxs[sensor_idx] = (range_idx + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  }
  
  sensor_idx = (sensor_idx + 1) % COUNT_SENSORS;
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

#ifdef GPIO_LOX1
  // If we defined GPIO pins, enable them as PULL UP
  pinMode(GPIO_LOX1, INPUT_PULLUP);
  pinMode(GPIO_LOX2, INPUT_PULLUP);
  pinMode(GPIO_LOX3, INPUT_PULLUP);
#endif

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  Serial.println("All in reset mode...(pins are low)");


  Serial.println("Starting...");
  setID();
}

void loop() {
  round_robin_read_sensors();

  for (int i = 0; i < COUNT_SENSORS; i++) {
    if (sensor_status[i] == VL6180X_ERROR_NONE) Serial.print(sensor_range_sums[i] / WINDOW_SIZE, DEC);
    else Serial.print("###");
    
    if (i != COUNT_SENSORS-1) Serial.print(" : ");
  }
  
  Serial.println();
  
  delay(25);
}
