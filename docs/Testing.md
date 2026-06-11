# Testing Log

What we tested, when we tested it, and what we found out. Some of these took one afternoon and some took weeks, because that is how it actually went. A few things were sorted out in one sitting and the rest took as long as they took.

---

## 10 June — Power distribution board

A simple check to see whether the distribution board was a reliable source of power, before we connected the rest of the electronics to it.

We measured both the 5V output and the full voltage output, first with nothing plugged in and then with the motor and the servo running, to see if the voltage dropped when something was pulling current. We wanted to know whether the board could hold a steady 5V while the motor was working hard. If it dipped at the exact moment the car started moving, we would have ended up chasing faults that looked like random software bugs.

It held steady, so we built the rest of the power system around it.
