#include <Wire.h>
#include <Arduino_RouterBridge.h>
#include <vector>
#include <zephyr/kernel.h>

// Standard Arduino scanner probes, with UNO Q Monitor instead of Serial.
// The additional RPC only retrieves results; it does not perform I2C operations.
static k_mutex resultsLock;
static int completed = 0;
static std::vector<int> results;
static std::vector<int> getResults() {
  k_mutex_lock(&resultsLock, K_FOREVER);
  std::vector<int> copy = {completed};
  copy.insert(copy.end(), results.begin(), results.end());
  k_mutex_unlock(&resultsLock);
  return copy;
}
void setup() {
  // Keep the existing three drivers' STEP outputs low throughout this test.
  for (int pin : {3,5,9}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  k_mutex_init(&resultsLock);
  Wire.begin();
  Bridge.begin();
  Monitor.begin();
  Bridge.provide("arduino_i2c_scan_results", getResults);
  Monitor.println("Arduino Wire I2C scanner: addresses 1 through 126");
}
void loop() {
  byte error, address;
  int nDevices = 0;
  std::vector<int> scan;
  Monitor.println("Scanning...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    scan.push_back(error);
    if (error == 0) {
      Monitor.print("I2C device found at address 0x");
      if (address < 16) Monitor.print("0");
      Monitor.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Monitor.print("Unknown error at address 0x");
      if (address < 16) Monitor.print("0");
      Monitor.println(address, HEX);
    }
  }
  if (nDevices == 0) Monitor.println("No I2C devices found");
  else Monitor.println("done");
  k_mutex_lock(&resultsLock, K_FOREVER);
  results = scan;
  completed++;
  k_mutex_unlock(&resultsLock);
  delay(5000);
}
