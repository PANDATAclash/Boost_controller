I built a simple electronic boost controller (EBC) using an Arduino Nano to control a MAC 3-port electronic boost solenoid with PWM.
The controller reads boost pressure from an analog MAP/boost sensor, and I set the target boost using a potentiometer. A 0.91" I2C OLED shows the current setpoint, measured pressure, and solenoid duty cycle.

This is the last version of the code I used (oled_smooth.ino) and I bench-tested it successfully—it does regulate pressure in the test setup.

Hardware I used (key parts)

Arduino Nano

MAC 3-port electronic boost solenoid

Analog MAP / boost pressure sensor

0.91" OLED (SSD1306, I2C, address 0x3C, 128×32)

Potentiometer for boost setpoint

Wastegate actuator / boost plumbing

Driver parts: I’m using a driver stage (transistor/MOSFET), a flyback diode, and basic resistors/caps. They are required to drive the solenoid reliably and protect the Arduino.

How my code works
Inputs / Outputs

Solenoid PWM: D5

MAP sensor: A0

Potentiometer: A1

OLED: I2C

Pressure reading + filtering

I read the MAP sensor voltage and convert it to pressure with a linear map based on:

0.2 V → 20 kPa

4.9 V → 250 kPa

Then I smooth the pressure using a low-pass filter (alpha = 0.1) to reduce noise.

Setpoint

I read the potentiometer and map it to a desired pressure between:

20 kPa → 250 kPa

PID control

I use the Arduino PID_v1 library, and the PID output is mapped to a PWM duty (0–255).

To avoid rapid toggling around the setpoint, I added hysteresis:

± 10 kPa around the setpoint

Safety features I added

MAP sensor failsafe: if sensor voltage drops below 0.2V, the controller disables boost control and shows SENSOR FAIL on the OLED.

Overboost protection: overboost limit is setpoint + 30 kPa (~0.3 bar).

If exceeded, the code triggers an overboost state and applies ~50% duty instead of fully shutting off, and shows OVERBOOST on the OLED.

It resets once pressure drops below (overboost threshold − hysteresis).

Display + Serial

OLED updates roughly every 100 ms (10 Hz).

Serial prints setpoint, pressure, and duty cycle at 115200 baud.

Current status (what is true)

 Bench tested: The controller works in a bench setup and responds to setpoint changes.
 In-car tuning needed: In a real turbo system it will need a lot of PID tuning and likely additional safeguards because vehicle conditions are much more dynamic and noisy.

Is the Arduino Nano “not fast enough”?

From my testing and based on how this code runs: the Arduino Nano is generally fast enough for this type of boost control, as long as the code isn’t blocking and the display isn’t updated excessively.

Most boost-control problems come from:

PID tuning and system dynamics (turbo/wastegate behavior)

sensor noise and filtering

solenoid frequency/strategy

wiring, grounding, and electrical noise

So I wouldn’t call the Nano “too slow” by default. A faster MCU only becomes necessary if I add heavy logging, advanced control strategies, or very high-rate control loops.

What can be improved next

PID tuning for real vehicle conditions (the biggest one)

Add hard limits (max duty, max boost cut strategy, sensor plausibility checks)

Improve pressure filtering and noise handling

Separate control loop timing from Serial printing even more

Add features like gear/RPM-based targets (optional)

Note (important bug to fix if needed)

Right now, the PID uses input, but I didn’t explicitly assign:

input = filteredPressure;


If I plan to use PID seriously in-car, I should set input to the filtered pressure before calling myPID.Compute() to guarantee the PID is using real sensor feedback.
