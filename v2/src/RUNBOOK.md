# Competition runbook — no Wi-Fi, no laptop help

Everything you need on the day, assuming there is no network at the venue and
nobody can SSH in from outside. Commands marked **UNTESTED** have not been run
on this Pi — try them at home before you travel.

Pi login: `pi` / sudo password `pi`. The Pi is called `ERROR404`.

---

## 1. Getting a terminal on the Pi with no Wi-Fi

Three ways, best first.

### a) The Pi makes its own hotspot, laptop joins it — **tested 2026-09-26**

Already set up on ERROR404 (NetworkManager profile `Hotspot`: SSID
**ERROR404**, password `wro2026team`, 2.4 GHz ch 6, Pi at 10.42.0.1). It is a
**fallback**: autoconnect priority -10, below every saved Wi-Fi, so at boot the
Pi joins a known network if one is in range and only starts the hotspot when
none is. Once on the hotspot it stays there until reboot or you switch.

Join Wi-Fi **ERROR404** from the laptop/phone, then:

```bash
ssh pi@10.42.0.1                 # NetworkManager's shared-mode address
# dashboard: http://10.42.0.1:8080/
```

Switch by hand (from a Pi terminal; your SSH drops when wlan0 changes mode):

```bash
sudo nmcli connection up Hotspot           # force the hotspot now
sudo nmcli connection up Testing           # back to the home Wi-Fi
sudo nmcli connection modify Hotspot connection.autoconnect no   # disable the fallback
nmcli connection show                      # list every saved connection
```

Recreate it from scratch if the profile is ever lost:

```bash
sudo nmcli device wifi hotspot ifname wlan0 con-name Hotspot ssid ERROR404 password wro2026team
sudo nmcli connection modify Hotspot 802-11-wireless.channel 6 ipv4.addresses 10.42.0.1/24 \
     connection.autoconnect yes connection.autoconnect-priority -10
```

⚠ The Pi has one radio: hotspot and home Wi-Fi never run at the same time.
`wro-radio.service` still kills Wi-Fi entirely when /boot/firmware/COMPETITION exists.

### b) Phone hotspot, both devices on it

Add the phone's hotspot to the Pi **before you travel** (it remembers it):

```bash
sudo nmcli device wifi connect "<HOTSPOT SSID>" password "<PASSWORD>"
nmcli -f NAME,AUTOCONNECT connection show
```

Then find the Pi from the laptop:

```bash
ssh pi@error404.local            # usually works
dns-sd -G v4v6 ERROR404.local    # if not: prints its current address (Mac)
```

⚠ Some phone hotspots isolate clients, so the laptop can see the network but
not the Pi. If that happens, use (a) or (c).

### c) Ethernet cable, laptop to Pi

No router needed — link-local addressing does it:

```bash
ssh pi@error404.local
```

On a Mac, set the Ethernet adapter to "Using DHCP" and give it a moment.

### d) Nothing works: keyboard + monitor

Micro-HDMI to the Pi, USB keyboard, log in as `pi`. Everything below works the
same; the dashboard is then `http://localhost:8080/`.

---

## 2. Services

`dashboard.service` owns the ESP32 serial port and the camera, and relays for
`main.py`. `master.service` runs `main.py`, the obstacle round. **If the
dashboard is not running, the car cannot move**, even if you never open the
web page.

```bash
systemctl is-active dashboard.service master.service     # are they up?
sudo systemctl restart dashboard.service master.service  # restart both
sudo systemctl stop master.service                       # stop driving, keep the dashboard
sudo systemctl stop master.service dashboard.service     # free the serial port (for flashing)
sudo systemctl start dashboard.service master.service
journalctl -u master.service -f                          # live log, Ctrl+C to leave
journalctl -u master.service -b --no-pager | tail -40    # this boot
```

Both start at boot on their own. The dashboard waits up to 30 s for the camera
first, so give it that long after power-on.

---

## 3. Which firmware is on the ESP32?

| Round | Firmware | Telemetry rate | Who drives |
|---|---|---|---|
| Open | `working_open_round` | ~8 Hz | the ESP32 alone, Pi ignored |
| Obstacle | `open_slave` | ~50 Hz | the Pi (`main.py`) |

Check without opening anything:

```bash
curl -s localhost:8080/api/state | python3 -c "import sys,json;s=json.load(sys.stdin);print(round(s['rate_hz'],1),'Hz')"
```

~8 Hz means the open round is loaded; ~50 Hz means the slave is.

---

## 4. Flashing the ESP32

### The easy way: the three buttons on the dashboard

The **Round** card has one button per round. Pressing one does everything:
compiles the sketch, hands the serial port to esptool, flashes the ESP32,
takes the port back, then starts the right Pi service and enables it at boot —
so the car comes back up in the same round after a power cut. About 25 s, and
the log under the buttons shows each step.

| Button | ESP32 firmware | Pi side | Telemetry |
|---|---|---|---|
| Open Round | `working_open_round` | nothing drives | ~3–8 Hz |
| Obstacle Round | `open_slave` | `master.service` | ~50 Hz |
| Parking test | `open_slave` | `parking.service` | ~50 Hz |

Open round is the default the car boots into: no parking, no `main.py`. A real
round never runs the parking service — that button is for testing the exit on
its own.

### By hand (all from the Pi)

Always stop the services first — they hold the serial port and the flash fails
mid-write otherwise.

```bash
export PATH=$HOME/bin:$PATH
cd ~/sketches
sudo systemctl stop master.service dashboard.service
```

**Open round** (ESP drives everything):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --output-dir build_wor working_open_round
cp build_wor/working_open_round.ino.merged.bin ~/esp_flash/
~/esp_flash/venv/bin/esptool --chip esp32c3 -p /dev/serial/by-id/usb-Espressif* -b 460800 \
    write-flash 0x0 ~/esp_flash/working_open_round.ino.merged.bin
```

**Obstacle round** (Pi drives):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --output-dir build_slave open_slave
cp build_slave/open_slave.ino.merged.bin ~/esp_flash/
~/esp_flash/venv/bin/esptool --chip esp32c3 -p /dev/serial/by-id/usb-Espressif* -b 460800 \
    write-flash 0x0 ~/esp_flash/open_slave.ino.merged.bin
```

**Make the car safe in a hurry** (motor held off, nothing else):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --output-dir build_stop motor_stop
cp build_stop/motor_stop.ino.merged.bin ~/esp_flash/
~/esp_flash/venv/bin/esptool --chip esp32c3 -p /dev/serial/by-id/usb-Espressif* -b 460800 \
    write-flash 0x0 ~/esp_flash/motor_stop.ino.merged.bin
```

Then bring the services back:

```bash
sudo systemctl start dashboard.service master.service
```

Flashing takes about 10 s. "Hash of data verified" means it worked.

---

## 5. Running a round

**Open round**: flash `working_open_round`, then just press START on the car.
The Pi is not involved. Corners are the smooth drift turn followed by a 90 cm
reverse.

**Obstacle round**: flash `open_slave`, make sure both services are up, then
press START. The button is a toggle — press again to stop, again to start a
fresh run. One run per power-up unless you press it again.

Before a scored run:

- Give the BNO055 a figure-eight until the dashboard calibration shows 3s.
  Every corner targets an absolute heading.
- Check the dashboard shows ToF values and the camera feed.
- `journalctl -u master.service -f` in a spare terminal tells you what it is
  deciding, live.

---

## 6. Tuning on the day

Sliders on the dashboard apply instantly; **Save to Disk** makes them survive a
restart. Without SSH you can still do all of it from a phone browser on the
hotspot.

From a terminal instead:

```bash
cat ~/obstacle_round/params.json                      # what is saved
python3 - <<'EOF'
import json; f="/home/pi/obstacle_round/params.json"
p=json.load(open(f)); p["post_turn_reverse_cm"]=90    # change what you need
json.dump(p,open(f,"w"),indent=2); print(p)
EOF
sudo systemctl restart master.service
```

The ones you are most likely to touch:

| Param | What it does |
|---|---|
| `post_turn_reverse_cm` | how far back after each corner |
| `corner_steer_deg` | corner steering cap — bigger is tighter |
| `front_turn_cm` | how close to the wall before turning |
| `pillar_trigger_dist_cm` | when the dodge commits |
| `dodge_short_cm` / `dodge_long_cm` | dodge leg lengths, normal and wide arc |
| `lap_direction` | **-1 anti-clockwise, 0 auto, +1 clockwise** — set it and the car never guesses |

---

## 7. Rule 11.10 — radios off for a scored run

```bash
sudo touch /boot/firmware/COMPETITION && sudo reboot     # Wi-Fi and Bluetooth off at boot
sudo rm /boot/firmware/COMPETITION && sudo reboot        # back to normal
```

With that file present the web dashboard does not start (the relay, camera and
YOLO still do, so the car drives normally) — and you cannot SSH in either, so
do your tuning first.

---

## 8. When something is wrong

```bash
# is anything crashing?
journalctl -u master.service -b --no-pager | grep -icE "traceback|AttributeError"
# is the ESP talking at all?
ls /dev/serial/by-id/
curl -s localhost:8080/api/state | python3 -m json.tool | head -20
# does the code still behave? (no hardware needed, ~20 s)
cd ~/obstacle_round && python3 tools/smoke_test.py
# last run, second by second
ls -t ~/obstacle_round/logs/run_*.csv | head -3
```

**ESP silent (0 Hz, no DATA lines)**: usually a ToF sensor stuck after a reset.
Power-cycle the whole car — a chip reset alone does not clear it.

**Camera missing**: it is only probed at boot. Reseat the USB camera and
reboot; `rpicam-hello --list-cameras` is for the CSI camera, which is dead on
this car.

**Put the code back to a known state**:

```bash
cd ~/wro_fe_26_codes && git log --oneline -10        # on the laptop
git checkout <commit> -- obstacle_round/             # then redeploy
ls ~/backups/                                        # on the Pi: dated copies
```

---

## 9. Before you leave home

- [ ] Pi hotspot: joined from laptop and phone (Pi side tested 2026-09-26)
- [ ] Phone hotspot saved on the Pi as a second option
- [ ] Ethernet cable packed, link-local SSH tested
- [ ] Both firmwares compiled on the Pi at least once (so `~/sketches/build_*`
      exists and a flash is 10 s, not a first-time toolchain download)
- [ ] `motor_stop` compiled, for the panic case
- [ ] Spare battery charged, and a spare USB cable for the ESP32
- [ ] `params.json` saved with the values you actually want
