void setup() {
      Serial.begin(115200); // USB Serial
      delay(1000);
      Serial.println("ESP32 ready over USB!");
    }

    void loop() {
      if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();
        if (msg.length() > 0) {
          Serial.print("ECHO: ");
          Serial.println(msg);
        }
      }
    }