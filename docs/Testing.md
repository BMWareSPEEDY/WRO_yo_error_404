# Testing Log

What we tested, when we tested it, and what we found out. Some of these took one afternoon and some took weeks, because that is how it actually went. A few things were sorted out in one sitting and the rest took as long as they took.

---

## 10 June — Power distribution board

A simple check to see whether the distribution board was a reliable source of power, before we connected the rest of the electronics to it.

We measured both the 5V output and the full voltage output, first with nothing plugged in and then with the motor and the servo running, to see if the voltage dropped when something was pulling current. We wanted to know whether the board could hold a steady 5V while the motor was working hard. If it dipped at the exact moment the car started moving, we would have ended up chasing faults that looked like random software bugs.

It held steady, so we built the rest of the power system around it.

---

## 11 June — Battery placement, and LiPo against Li-ion

Two things on the same day. The first was working out where the battery should sit. We moved the pack around the chassis and drove the car each time to see how the weight changed the way it handled. Weight over the back wheels helps the car grip, but too much of it makes the steering feel loose.

The second was choosing between a LiPo battery and three Li-ion cells. We wrote out the good and bad points of each one, looking at how fast they can give out current, how easy they are to mount, how easy they are to charge, and how safe they are to store between sessions. We went with the Li-ion cells. The reasons are written up in [`Decisions.md`](./Decisions.md).

---

## 25 June to 15 July — Open round testing

Three weeks of running the car on the practice track over and over, which is where most of the open round work actually happened.

Every session went the same way. Run the car, watch where it went wrong, work out whether the problem was mechanical, electrical or in the code, fix it, then run it again. A lot of what we found was not what we expected. Problems that looked like bad code often turned out to be a sensor reading late or a wire coming loose, and problems that looked electrical were sometimes the code reacting to a wall it should have ignored.

The steering took the longest by far. We kept changing the turning angles and the distance at which the car decides to turn, running it, then changing them again, until the turns were tight enough to get round the corner without clipping the wall. There was no way to work those numbers out on paper. The track gave them to us, one run at a time.

---

## 20 July to 1 August — Raspberry Pi camera and operating system

Once the Pi 5 was in, this stretch was about making the camera and the link between the two boards reliable enough to build on.

We tested the camera constantly to check that the video feed kept running instead of freezing or dropping frames partway through, and we worked through the Raspberry Pi 5 operating system setup until the Pi and the ESP32 talked to each other consistently. Being reliable was the whole point here. A camera system that works nine times out of ten is no use, because the tenth run is the one that happens in front of the judges.

---

## 1 August onwards — HSV tuning and detection

Ongoing testing and tuning of the Raspberry Pi code, to make the car's colour detection something we can trust.

Most of this is HSV tuning, which means changing the colour ranges until the red and green pillars and the magenta parking walls show up clearly without the background getting picked up as well. Along with that we have been sorting out the regions of interest, so the car only looks at the part of the picture where pillars actually turn up and ignores the rest, and checking that the video processing keeps up with the driving code instead of falling behind.

This one is still going and will keep going up to the competition. The WRO rules say the real colours on the field can be different from the printed ones, and teams get practice time at the venue for exactly this reason. So the values we have now are a starting point that we will tune again under the lighting on the day.
