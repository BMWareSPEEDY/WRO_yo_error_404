# Design Decisions

This is a list of the choices we made while building the car, and why we made them. They are in the order we made them. A couple of them we got wrong the first time and had to change later, and we left those in as well.

---

## 10 June — Power distribution board instead of a buck converter

**Decision.** Use a power distribution board to split up the battery power, instead of a buck converter.

**Why.** One battery has to power a few different things, and they do not all want the same voltage. The motor driver needs the full battery voltage. The ESP32, the sensors and the servo all need 5V. A buck converter only gives you one output, so every extra thing you plug in becomes another join in the wiring.

The distribution board has 5V pads and full voltage pads on the same board, so everything has its own place to connect. This made the wiring a lot neater. That mattered more than we thought it would, because most of the electrical problems we had later were loose or bad connections rather than broken parts, and fewer joins means fewer places for a bad connection to hide. It also left us room to add more parts later, which helped when the Raspberry Pi arrived.

---

## 11 June — Three Li-ion cells instead of one LiPo battery

**Decision.** Power the car with three 18650 Li-ion cells in series, which gives about 11.1V, instead of a single LiPo battery.

**Why.** A LiPo can give out current faster, and for a car that is mostly running one motor that is a real advantage, just a small one. What made up our minds was everything around the battery instead of the battery itself. The 18650 cells sit in a holder that bolts straight to the chassis, we can swap out one cell if it goes bad instead of replacing the whole pack, and they are much easier to charge and store between build sessions. LiPo packs need more care when charging and are not something we wanted sitting in a school bag between meetings.

Three cells in series also gave us a good voltage to work with. It is high enough for the motor to have proper torque, and low enough that the distribution board can drop it to 5V without getting hot.

---

## 18 June — Raspberry Pi camera instead of a USB camera

**Decision.** Use a Raspberry Pi camera on the ribbon connector instead of a USB webcam.

**Why.** In the obstacle round everything depends on how fast the car can see a pillar and react to it. Any delay in getting the picture is a delay in the whole system. A ribbon camera plugs straight into the Pi's camera hardware, which is faster and more consistent than a USB webcam.

The other reason was the USB port itself. The ESP32 is already connected over USB, and we did not want the camera and the link to the motor controller sharing the same connection during a run. Putting the camera on the ribbon connector means USB only has one job to do.
