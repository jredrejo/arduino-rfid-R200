/**
 * R200 RFID demonstration
 * Copyright (c) 2023 Alastair Aitchison, Playful Technology
 */

// INCLUDES

#include "R200.h"

// GLOBALS
unsigned long lastResetTime = 0;
R200 rfid;

void setup() {

  // Intitialise Serial connection (for debugging)
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);

  rfid.begin(&Serial2, 115200, 16, 17);

  // Get info
  rfid.dumpModuleInfo();

  // Test functions
  tests();
}

void loop() {
  rfid.loop();

  // Periodically re-send the read command
  if(millis() - lastResetTime > 1000){
    //  digitalWrite(LED_BUILTIN, HIGH);
    rfid.poll();
    //rfid.dumpUIDToSerial();
    //rfid.getModuleInfo();
    //  digitalWrite(LED_BUILTIN, LOW);
    lastResetTime = millis();
  }
  delay(1000);
}


void tests() {


    // ============================================
    // TEST: Get and display current power
    // ============================================
    Serial.println("\n===== Testing getPower() =====");
    float currentPower = rfid.getPower();
    if(currentPower >= 0) {
      Serial.print("Current transmit power: ");
      Serial.print(currentPower);
      Serial.println(" dBm");
    } else {
      Serial.println("Error: Failed to read power");
    }
    delay(500);

    // ============================================
    // TEST: Set power to a specific value
    // ============================================
    Serial.println("\n===== Testing setPower() =====");
    float newPower = 20.0; // Set to 20 dBm
    Serial.print("Setting transmit power to: ");
    Serial.print(newPower);
    Serial.println(" dBm");

    if(rfid.setPower(newPower)) {
      Serial.println("Power set successfully!");

      // Read back the power to verify
      delay(500);
      float verifyPower = rfid.getPower();
      if(verifyPower >= 0) {
        Serial.print("Verified power: ");
        Serial.print(verifyPower);
        Serial.println(" dBm");
      }
    } else {
      Serial.println("Error: Failed to set power");
    }
    delay(500);

    // ============================================
     // TEST: Get demodulator parameters
     // ============================================
     Serial.println("\n===== Testing getDemodulatorParams() =====");
     uint8_t mixer_g = 0;
     uint8_t if_g = 0;
     uint16_t thrd = 0;

     if(rfid.getDemodulatorParams(mixer_g, if_g, thrd)) {
       Serial.println("Demodulator parameters:");
       Serial.print("  Mixer Gain (Mixer_G): ");
       Serial.println(mixer_g);
       Serial.print("  IF Gain (IF_G): ");
       Serial.println(if_g);
       Serial.print("  Threshold (Thrd): ");
       Serial.println(thrd);
     } else {
       Serial.println("Error: Failed to read demodulator parameters");
     }
     delay(500);

     // ============================================
     // TEST: Set demodulator parameters
     // ============================================
     Serial.println("\n===== Testing setDemodulatorParams() =====");
     uint8_t new_mixer_g = 2;
     uint8_t new_if_g = 6;
     uint16_t new_thrd = 176;

     Serial.print("Setting demodulator parameters to: Mixer_G=");
     Serial.print(new_mixer_g);
     Serial.print(", IF_G=");
     Serial.print(new_if_g);
     Serial.print(", Thrd=");
     Serial.println(new_thrd);

     if(rfid.setDemodulatorParams(new_mixer_g, new_if_g, new_thrd)) {
       Serial.println("Demodulator parameters set successfully!");

       // Read back the parameters to verify
       delay(500);
       uint8_t verify_mixer_g = 0;
       uint8_t verify_if_g = 0;
       uint16_t verify_thrd = 0;

       if(rfid.getDemodulatorParams(verify_mixer_g, verify_if_g, verify_thrd)) {
         Serial.println("Verified demodulator parameters:");
         Serial.print("  Mixer Gain: ");
         Serial.println(verify_mixer_g);
         Serial.print("  IF Gain: ");
         Serial.println(verify_if_g);
         Serial.print("  Threshold: ");
         Serial.println(verify_thrd);
       }
     } else {
       Serial.println("Error: Failed to set demodulator parameters");
     }

     Serial.println("\n===== All tests completed =====\n");

}
