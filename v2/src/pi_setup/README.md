# Pi setup (copied from the Pi on 2026-09-19)

| File | Installed at | Purpose |
|---|---|---|
| `dashboard.service` | `/etc/systemd/system/dashboard.service` | Starts **`dashboard.py`** at boot and keeps it running whether `main.py` runs or not. It owns the ESP32 port and the camera, serves `http://error404.local:8080/`, and relays for `main.py`. In competition mode the web page and camera are off but the relay still runs. |
| `master.service` | `/etc/systemd/system/master.service` | Starts **`/home/pi/obstacle_round/main.py`** (the Pi master) at boot (after `dashboard.service`) and restarts it if it exits. It keeps only the 20 newest run CSVs. Stopping it (`sudo systemctl stop master.service`) leaves the dashboard up. The previous unit, which ran the V1 `/home/pi/codes/master.py`, is saved on the Pi as `master.service.bak.v1_codes_master`. |
| `deploy.sh` | (run on the laptop) | Deletes the old obstacle code from the Pi (`~/obstacle_round` code and `~/codes`) after saving a tarball in `~/scrapped_obstacle_*.tar.gz`, and keeps the dataset, runs and models. Then it copies `obstacle_round/pi/` to `~/obstacle_round/`, installs both services, and enables and restarts them. |
| `wro-radio.service` | `/etc/systemd/system/wro-radio.service` | Runs `wro-radio.sh` once at boot |
| `wro-radio.sh` | `/usr/local/bin/wro-radio.sh` | **Rule 11.10.** If `/boot/firmware/COMPETITION` exists, it blocks Wi-Fi and Bluetooth with `rfkill`. |

Race-day flow: power on, wait about 30 s for the Pi to boot (`main.py` is then in WAITING), then press START.

Live log: `journalctl -u dashboard.service -u master.service -f`. Stop it for bench work: `sudo systemctl stop master.service`.

For a scored round, put the SD card in a laptop and create an empty file named `COMPETITION` on the boot partition. Delete it again to get Wi-Fi and SSH back.

Install on a fresh Pi:

```bash
sudo cp dashboard.service master.service wro-radio.service /etc/systemd/system/
sudo cp wro-radio.sh /usr/local/bin/ && sudo chmod +x /usr/local/bin/wro-radio.sh
sudo systemctl daemon-reload && sudo systemctl enable wro-radio.service dashboard.service master.service
```
