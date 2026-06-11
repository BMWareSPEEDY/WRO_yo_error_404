# Design Decisions

This is a list of the choices we made while building the car, and why we made them. They are in the order we made them. A couple of them we got wrong the first time and had to change later, and we left those in as well.

---

## 10 June — Power distribution board instead of a buck converter

**Decision.** Use a power distribution board to split up the battery power, instead of a buck converter.

**Why.** One battery has to power a few different things, and they do not all want the same voltage. The motor driver needs the full battery voltage. The ESP32, the sensors and the servo all need 5V. A buck converter only gives you one output, so every extra thing you plug in becomes another join in the wiring.

The distribution board has 5V pads and full voltage pads on the same board, so everything has its own place to connect. This made the wiring a lot neater. That mattered more than we thought it would, because most of the electrical problems we had later were loose or bad connections rather than broken parts, and fewer joins means fewer places for a bad connection to hide. It also left us room to add more parts later, which helped when the Raspberry Pi arrived.
