# Pi master: open round (3 laps, no pillars)

| File | What it does |
|---|---|
| `main.py` | Control loop. It uses the same logic and numbers as `open_round/`: heading P + wall centring on straights, a corner at front ≤ 60 cm, a fixed-lock turn until the heading is 90° on, and a reverse if wedged. The direction is locked at the first corner. After 12 turns it drives 3 s more, then stops. Each run writes a CSV to `logs/`. |
| `config.py` | Every constant. Speed is capped at **170 PWM** (`MAX_PWM`) and steering at **30..140**. |
| `serial_link.py` | `SerialLink` owns the USB port and is used by the dashboard. `HubLink` is what `main.py` uses: it goes through the dashboard over UDP on localhost. |
| `vision.py` | YOLO pillar detector (`models/wro_det320.onnx`, 320 px, onnxruntime) plus a background worker. It runs in `dashboard.py` on every camera frame, about 18 fps on the Pi 5. Each block has its class, confidence, box, x and a rough distance (`PILLAR_FOCAL_PX` is not calibrated yet). |
| `camera.py` | Threaded capture, owned by the dashboard: `CameraStream` (CSI camera via picamera2) and `UsbCameraStream` (USB/UVC webcam via OpenCV V4L2, MJPG). `open_camera()` picks one - `config.CAMERA_BACKEND` "auto" tries CSI then USB. The car drives without a camera. |
| `dashboard.py` | **Its own service, running from boot whether or not `main.py` runs.** It is the only program that opens the ESP32 port and the camera, and it relays ESP32 lines and commands for `main.py`. Page at `http://error404.local:8080/`: camera stream with the YOLO boxes drawn on it, a Pillars card (fps and a table of what it sees), the 4 ToF values, a 15 s ToF graph, yaw/target/error, IMU calibration, the Pi's command vs what the ESP applied, link rate/age and the log. With `/boot/firmware/COMPETITION` present, the page and camera are off and only the relay runs. |

Both start at boot after `pi_setup/deploy.sh`. Then press START to drive.
Stop driving but keep the dashboard: `sudo systemctl stop master.service`. Start again: `sudo systemctl start master.service`, or `python3 main.py`.
