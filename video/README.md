# Vehicle Testing Videos (`/video`)

This directory tracks the driving test footage for the **ERROR 404** autonomous vehicle. Full runs are uploaded to YouTube and linked below so they can be viewed directly without downloading any files.

---

## 🎥 Video Inventory

- **[Open Challenge — Testing Run](https://www.youtube.com/watch?v=eD4r7zBw4Pc)**:
  A full autonomous test of the Open Round routine — ultrasonic wall centering, IMU-held straights, 90° corner turns, and lap counting all running together on the practice track. The ESP32 runs the entire vehicle on its own in this round.

- **[Obstacle Challenge — Testing Run](https://www.youtube.com/watch?v=VzUp-tTwaB4)**:
  A full autonomous test of the Obstacle Round, with the Raspberry Pi 5 as master and the ESP32 as slave. The camera drives the decisions here: red and green pillars are detected in HSV and passed on the rule-mandated side, corner turns are gated on the frame being clear of pillars, and the BNO055 squares the heading up to the nearest 90° after each turn.

Both runs are fully autonomous — the vehicle is started with the push button and nothing is touched after that.

---

