#!/bin/bash
# deploy.sh - put obstacle_round/pi (Pi master, open round + dashboard) on the Pi
# and make it start at boot. Run on the laptop:   pi_setup/deploy.sh
#
# 1. deletes the old obstacle-round code from the Pi (~/obstacle_round code,
#    ~/codes V1 master.py). A tarball of it is left in ~/scrapped_obstacle_*.tar.gz
#    first. Training data (dataset, runs, models, .venv_train) is kept.
# 2. rsync obstacle_round/pi/ -> ~/obstacle_round/
# 3. installs dashboard.service (dashboard + ESP32 relay, always on) and
#    master.service (main.py), enables + restarts both
set -e
PI=${PI:-pi@error404.local}
REPO=$(cd "$(dirname "$0")/.." && pwd)

# Sync user's tuned params.json from Pi back to laptop repo first so it is never lost
rsync -a "$PI:/home/pi/obstacle_round/params.json" "$REPO/obstacle_round/pi/params.json" 2>/dev/null || true

ssh "$PI" 'set -e
  cd ~
  stamp=$(date +%Y%m%d_%H%M%S)
  echo pi | sudo -S -p "" systemctl stop master.service dashboard.service 2>/dev/null || true
  old=""
  [ -d obstacle_round ] && old="$old $(cd ~ && ls -d obstacle_round/* obstacle_round/.[!.]* 2>/dev/null \
      | grep -vE "/(dataset|dataset.zip|runs|models|logs|\.venv_train|params\.json|params\.working_backup\.json)$" | tr "\n" " ")"
  [ -d codes ] && old="$old codes"
  if [ -n "$(echo $old | tr -d " ")" ]; then
    tar czf scrapped_obstacle_$stamp.tar.gz $old
    rm -rf $old
    echo "old obstacle code removed (backup: ~/scrapped_obstacle_$stamp.tar.gz)"
  fi
  mkdir -p obstacle_round/logs'

# Transfer code while strictly preserving params.json on the Pi
rsync -a --exclude params.json --exclude params.working_backup.json --exclude __pycache__ --exclude logs --exclude .DS_Store \
  "$REPO/obstacle_round/pi/" "$PI:/home/pi/obstacle_round/"
# Only copy params.json if it does not already exist on the Pi
rsync -a --ignore-existing "$REPO/obstacle_round/pi/params.json" "$PI:/home/pi/obstacle_round/params.json" 2>/dev/null || true
rsync -a --ignore-existing "$REPO/obstacle_round/pi/params.working_backup.json" "$PI:/home/pi/obstacle_round/params.working_backup.json" 2>/dev/null || true

scp -q "$REPO/pi_setup/master.service" "$REPO/pi_setup/dashboard.service" "$PI:/tmp/"
ssh "$PI" 'set -e
  python3 -c "import serial, cv2, picamera2" || echo "WARNING: missing python package (pyserial / opencv / picamera2)"
  echo pi | sudo -S -p "" cp /tmp/master.service /tmp/dashboard.service /etc/systemd/system/
  echo pi | sudo -S -p "" systemctl daemon-reload
  echo pi | sudo -S -p "" systemctl enable dashboard.service master.service >/dev/null 2>&1
  echo pi | sudo -S -p "" systemctl restart dashboard.service master.service
  sleep 6
  for u in dashboard master; do echo "$u.service: $(systemctl is-enabled $u.service), $(systemctl is-active $u.service)"; done
  journalctl -u dashboard.service -u master.service -n 14 --no-pager | cut -c1-170
  ls ~/obstacle_round'
echo "dashboard: http://${PI#*@}:8080/"
