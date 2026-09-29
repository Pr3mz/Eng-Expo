# Post-Mortem & Lessons Learned (v1.2.0 -> v1.3.0)

## The Mistake: DC Motor Stall Torque (Deadband) Ignored
In the previous prompt, you requested:
> "dont make the robot make the desicion for speed just make it cosntant speed like moving fordward 100 turning degress maybe 60 power"

**What I did wrong:** I blindly followed the exact numbers requested (`100` for forward, `60` for turning) and completely removed the `clamp_speed()` safety function that was preventing the PWM values from dropping below the motor's minimum threshold. 

**Why it caused failure:** In reality, generic DC motors (like the ones driven by the L298N or similar H-bridges) have physical friction in their gearboxes. They typically will not spin at all if the PWM signal is below ~`80` to `110`. By sending exactly `60` power to turn, the motors were receiving electricity, but not enough to overcome physical friction, causing the motors to simply hum and stall (which makes the robot look like it's completely dead or lagging).

## How I Fixed It
1. I restored the `clamp_speed()` function to act as a **physical hardware safety** rather than an AI decision. If any code requests a speed greater than 0, the clamp ensures it gets bumped to at least `110` so the motors actually physically move.
2. If you request `60` power, the clamp will transparently bump it up to the bare minimum required to turn the wheel (`110`), ensuring the robot remains responsive without stalling out on the mat.
3. I also kept the new ESP32 LERP profile (accelerating by `15` PWM every `10ms`) which eliminates the wheel slip/wheelie problem without introducing the massive 500ms latency we saw in version 1.1.0.

## Takeaway for Future Prompts
When working with hardware, always decouple "AI logical speed" from "Hardware electrical minimums." Even if a user asks for a very slow speed, the code must still enforce the physical stall-torque minimums (deadband) of the specific DC motors being used.
