#!/bin/bash
# Rule 11.10: no RF/Bluetooth/Wi-Fi during scored rounds, and judges may
# inspect. Presence of /boot/firmware/COMPETITION kills every radio at boot.
#
# The boot partition is FAT32, so this file can be created or deleted from any
# laptop with the SD card in a reader - no SSH, no keyboard, no monitor. That
# matters because once the radio is off you cannot reach the Pi to undo it.
if [ -f /boot/firmware/COMPETITION ]; then
    rfkill block wifi      || true
    rfkill block bluetooth || true
    logger -t wro-radio "COMPETITION mode: wifi + bluetooth BLOCKED (rule 11.10)"
else
    rfkill unblock wifi    || true
    logger -t wro-radio "debug mode: wifi enabled"
fi
