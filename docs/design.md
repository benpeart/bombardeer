# Bombardeer

# Features

* Turret will automatically identify targets of interest (deer) and shoot them.  
* Need to record video of shots so we can get cool footage  
* To minimize the number of turrets needed to protect a given area, greater range is always nicer  
* Ability to track moving targets and fire on the move  
* Ability to track multiple targets and rapidly switch between them while firing  
* Remote control/view

# MVP

* Single camera, no depth calculations, no ballistics, just point and shoot at the center of the object  
* Pan and tilt to aim turret  
* Auto fire when on target  
* Implement Ring-Buffer / Event-Based Recording that will record/save video x seconds before and after firing

# Hardware Design

## Turret Base

* Need a gun mount that can point the gun in any given direction and angle (rotate left/right 180 degrees, up/down \~45 degrees?) quickly and quietly.  
  * Gun mount has pan and tilt support using [stepper motors](https://www.amazon.com/dp/B00PNEQKC0?ref=ppx_yo2ov_dt_b_fed_asin_title) driving a small gear that engages with a geared turntable (ala Lego Technic Turntable).   
  * Gives us the ability to change the gear ratio to ensure we have the strength to turn/aim the gun assembly.  
* Commercial transit tripod base.   
  * Allows you to easily move it around plus can level it on uneven ground.  
  * [Amazon.com : transit tripod](https://www.amazon.com/s?k=transit+tripod)  
* [T Slot 2020 Aluminum Extrusion](https://www.amazon.com/dp/B0B7HQRQ5G?th=1) for frame  
  * Strong, relatively light, versatile, weather proof  
  * [M5 T Nut Screws Kit for 2020 Aluminum Extrusion](https://www.amazon.com/dp/B0DMCS8KW4?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)   
* Pan control: Use outer ring geared [Slewing bearing](https://en.wikipedia.org/wiki/Slewing_bearing) to support rotating parts  
  * [Amazon.com: ZCLGOOD 6 Inch Black Lazy Susan Turntable](https://www.amazon.com/dp/B0FPKTKY5C?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1) with a 3D printed gear bolted to it  
  * [Amazon.com: STEPPERONLINE Nema 17 Stepper Motor](https://www.amazon.com/dp/B00PNEQKC0?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)   
  * [Amazon.com: STEPPERONLINE Mounting Plate and Bracket for Nema 17 Stepper Motor](https://www.amazon.com/dp/B09LCSBLF8?ref=ppx_yo2ov_dt_b_fed_asin_title)  
* Tilt control: To support rotating top rail for adjusting tilt  
  * Mount two [Pillow Block Bearings](https://www.amazon.com/Eowpower-Diameter-Mounted-Pillow-Bearing/dp/B07K68Z1RC) (e.g., KP08 or KFL08, \~$5–$10 each) onto the vertical [T-slot uprights](https://www.amazon.com/dp/B0B7HQRQ5G?th=1) on either side of the turret base.  
  * [Amazon.com: 8mm x 100mm 304 Stainless Steel Round Rod](https://www.amazon.com/dp/B0D5D3KFQX?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)   
  * Attach Shaft Collars or custom T-slot mounting plates to lock the tilting top rail directly onto that shaft.  
  * [Amazon.com: STEPPERONLINE Nema 17 Stepper Motor](https://www.amazon.com/dp/B00PNEQKC0?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)   
  * [Amazon.com: DC Stepper Motor 42 Single Row Straight Plate Mounting Bracket](https://www.amazon.com/dp/B0GCCC4F81?ref=ppx_yo2ov_dt_b_fed_asin_title)   
  * [Amazon.com: BLCCLOY 20Pcs 2020 Aluminum Extrusion Black Corner Bracket T Slot](https://www.amazon.com/dp/B08D6T9CGN?th=1)  
* How things stack up from top to bottom:  
  * Horizontal T-slot with 8mm steel rod at each end run through pillow bushing  
  * Vertical T-slot arms holding pillow bushings  
  * horizontal t-slot attached to turnable with regular t-slot fasteners  
  * 3D printed turntable gear  
    * recessed holes for bolts to go up and into t-slot above  
    * heat set inserts to attach to turntable bearing below  
  * Turntable bearing (lazy suzan) attached to t-slot base with regular t-slot fasteners  
  * Square t-slot base  
* How to mount and fire the gun?  
  * 3D print mount that screws directly into handle screws of paint ball gun and slips over top rail  
  * Mount has holes for Heschen HS-1564B solenoid with bolt through end to contact trigger  
  * [Heschen DC24V 1.5A DC Solenoid Electromagnet](https://www.amazon.com/dp/B09T2XVMQJ?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)  
* Power Supply  
  * [Amazon.com: PlusRoc 2-Pack Waterproof DC 12V/24V to 5V 5A Buck Converter, USB-C Output](https://www.amazon.com/dp/B0FD735LFG?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)  
  * [Amazon.com: Battery Terminal Connectors 1 Pair](https://www.amazon.com/dp/B0BV3628Q7?ref=ppx_yo2ov_dt_b_fed_asin_title)   
* How to weatherproof stepper motors? Electronics?  
  * 3D printed enclosure with ‘lid’ on the bottom so water won’t get in

## Trigger Circuit

To safely drive the [**Heschen HS-4564B 24V solenoid (1.5A)**](https://www.amazon.com/dp/B09T2XVMQJ?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1) from the ESP32 (3.3V logic on GPIO16), we need a switching circuit (N-channel MOSFET) with proper flyback protection and gate drive resistors to isolate the 3.3V logic from the 24V inductive load.

[MOSFET Control Module, 30V 161A: Amazon.com: Industrial & Scientific](https://www.amazon.com/dp/B0CTMB6BQX?ref=ppx_yo2ov_dt_b_fed_asin_title) 

## Deterrant

## Paintball Gun

* [I Built an Xbox-Controlled Paintball Turret](https://www.youtube.com/watch?v=UVgqzoGp9NQ)   
* [Amazon.com : Action Village Kingman Spyder Victor Entry Paintball Gun Package Kit (Diamond Black) : Sports & Outdoors](https://www.amazon.com/Action-Village-Kingman-Paintball-Package/dp/B0BHXNYJYY?crid=144S5OI0867OZ&dib=eyJ2IjoiMSJ9.Iy9bidGex7JaozakTUS2uqiunbXC9p784XkwueAryM7NmMcDGgLH25FxrqiFMYtLOVPld20ooZX0Nk4CZVRcRGu3WtGE1W40hit2Q_t36j7c408Mcwil6SHkIbA-WAAuBCRmhiU4jn0KL5zj-ox1G8OCGnnEJ5LgMXJJdeAzbijp-Qh1q4XH4NS6mA6GW1uNhT23Tgxk_a3_IOA8_jfjCusLiJv4Seq4QFjHeofyNa6fz4wvcijwDMzMIM3tXZ6jmw8bYmD7-F-Ld5NI6eNJQnFEJurupSXtUlQ9flDvjWU.BDgzZbN8qCrVGXsO1n3FZ0gRckiPLPaNFFX-0CawXmU&dib_tag=se&keywords=paintball%2Bpistol&qid=1785462203&s=sporting-goods&sprefix=paintball%2Bpistol%2Csporting%2C135&sr=1-22&th=1)   
* Usable range is 25-35 yards, max 50 yds

## 

  ## Non-Kinetic Options

Given that the civil/legal liability of firing actual paintballs in a suburban setting remains a significant barrier, replace the paintball gun with high-intensity directional payloads:

| Payload Type | Mechanism | Why It Works |
| :---- | :---- | :---- |
| **High-Pressure Water Solenoid** | 12V quick-response solenoid valve tied to a tight jet nozzle. | Delivers a hard physical shock/thump at 40–60 PSI without any legal liability. |
| **Ultrasonic / Directional Audio** | Targeted speaker playing wolf/hound barks or high-frequency distress calls. | Directional speakers restrict sound to the target area without waking neighbors. |
| **Strobe / Green Laser Pen** | Green laser module aimed lower than human eye-level. | Deer visual systems are exceptionally sensitive to green light wavelength contrast in pitch dark. |

## Airsoft Gun

* [Amazon.com : Game Face GF76 Electric Full/Semi-Auto Tactical Carbine Airsoft Rifle](https://www.amazon.com/dp/B00C87XZ5U?tag=riflepal-20&linkCode=ogi&th=1&psc=1#customerReviews)  
* [Automatic Electric Gun](https://airsoft2day.com/how-does-an-electric-airsoft-gun-work/) (AEG) are airsoft guns that are powered by an electric battery  
  * In summary, electric airsoft guns are powered by a battery that drives a motor, which compresses a spring and creates a burst of air that propels the BB out of the barrel. The firing mechanism is activated by a switch that sends an electrical current to the motor, while the loading mechanism uses a magazine to store and feed BBs into the gun.  
* Need a way to store enough ammunition protected from the elements and deliver it reliably to the firing chamber  
  * Airsoft typically have magazines that hold 150+ rounds (the ammo is much smaller as well)  
    * High capacity and drum magazines can hold significantly larger numbers of BBs  
  * I believe airsoft magazines and the smaller/harder ammo (6mm) will be less likely to bridge/fail to load but we should research this issue.  
* Need to be able to programmatically fire the gun  
  * We can take advantage of the AEG wiring and replace the trigger switch with a circuit driven by an embedded controller (Teensy, ESP32, etc) [to trigger the MOSFET and cause the gun to fire](https://schematron.org/airsoft-mosfet-wiring-diagram.html). This is fairly common practice in various electronic projects.  
* Airsoft guns are typically accurate to 50m with upgraded sniper rifles up to 100m  
  * More range is better as each turret can cover a greater area. It also means the sound they make to rotate the gun into position and fire is less likely to spook the deer.  
* Must be able to correctly calculate barrel position and angle  
  * The most simple/direct solution would be to use an [9DoF accelerometer/gyro/compass](https://www.adafruit.com/product/4517) attached to the gun directly that can be used to detect/compute the current orientation. Accuracy will depend entirely on the absolute accuracy of the 9 DoF IMU used.  
  * Could be accomplished by mounting the turret on a tripod with bubble levels and manually ensuring it is level. Then we use kinematics to compute Euler angles (roll, pitch and yaw) as we move the gun via motors/rotary encoders.  
    * May be able to use StallGuard feature of TMC2209 to calibrate and detect if/when stepper motors can’t turn and the position is unknown.  
    * May need to have rotary encoders to track gun movement (vs what was requested)  
    * Leveling of the turret can be automated with an MPU6050 and servo motors on each leg but this is complex and expensive (see the $500 [Benro Theta: the Intelligent Modular Travel Tripod | Benro](https://www.benro.com/en/product/benro-theta.html)).  
* Need to detect when the gun fails to shoot and shut down gracefully.

## Computer

* Raspberry Pie 5 8GB ($80)  
  * Need to determine how much compute is required and if this can support it.  
  * [Hailo-8L AI Kit 26 TOPS ($120)](https://www.seeedstudio.com/Raspberry-Pi-Al-HAT-26-TOPS-p-6243.html)  
* [NVIDIA Jetson Orin Nano](https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/) ($500)?  
  * [plertvilai/birdCam\_jetson: BirdCam with Jetson Nano](https://github.com/plertvilai/birdCam_jetson)

## Vision

* Mount camera(s) on moving gun support so it tracks where the gun is aiming  
* Need a way to see the entire area within range, scan for and identify targets.  
  * Wide angle lens to see wide field of view and eliminate scanning  
  * Will need IR/thermal cameras so they work at dawn/dusk and in the dark.  
  * Will likely need IR illuminators to get desired range  
  * Need to be weatherproof  
    * [Wildlife Cam Case – naturebytes kit store](https://shop.naturebytes.org/products/wildlife-cam-case-by-naturebytes)  
  * May be able to use [security cameras](https://www.axis.com/products/axis-p4705-plve) as they have similar requirements  
* Cameras  
  * [Simplifying embedded vision for all. \- Arducam](https://www.arducam.com/)  
  * [Camera \- Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)  
* Need a way to compute the position of the selected target in 3D space.  
  * Depth camera? Lidar? Stereo Cameras?

