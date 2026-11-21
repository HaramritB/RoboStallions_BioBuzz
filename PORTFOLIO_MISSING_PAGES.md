# BioBuzz portfolio: content for the missing pages

Sources: the 18-page portfolio PDF, the 2-page trifold PDF, and this repo (`Robot.java`, `Constants.java`,
`AUTONOMOUS.md`, `V2.md`, `SHOOTER_TUNING.md`, `TURRET_TRACKING.md`, `git log`).

Key:
- **READY** = taken or computed from the code, trifold or portfolio. Paste it in.
- **MEASURE** = no number exists in the repo or the PDFs, so I did not invent one. The how-to is given.
- **CONFIRM** = my draft wording that the team should check against what really happened.

---

## 0. What is missing, page by page

| PDF page | Printed # | What is missing |
| --- | --- | --- |
| 9 | 9 | Autonomous / TeleOp / Strategy goal boxes, design brainstorming bullets and conclusion, 2 images |
| 10 | 10 | Iteration 1 "Built for" and stats, Iteration 2 results and 3 stats, 3 Problem Solving (Lesson / Redesign) boxes |
| 11 | 11 | Drivetrain "give me a number" and graph/calcs |
| 12 | 12 | Intake goals are copy-pasted from Drivetrain. V1 and V2 bullets and "Stats, math, notes" are empty |
| 13 | 13 | Entire page blank ("INTAKE CONT.") |
| 14 | 13 (labelled "INTAKE CONT." again) | Needs images. Body text runs over the "Design challenges and iterations" heading |
| 16 | 15 | "replace this" under the turret bullets |
| 18 | none | Looks like an older duplicate of Robot Overview with no page number |
| not present | | **TeleOp**, **Onboard processing / electronics**, **Gate / feeder / flower**, **Think (goals, accomplishments, learning plan)** |

The judge's guide on page 2 promises CONTROL = "Autonomous and Teleop Solutions, Onboard Processing" and
THINK = "Goals, Accomplishments, and Learning Plans, Lessons Learned". Only Autonomous exists, so sections 8, 10 and 11 below are the
most important to add. The guide's page ranges (Innovate 9-15, Control 15-16) also no longer match the PDF.

---

## 1. Page 9: Technical Process

### Objective brainstorming boxes

**AUTONOMOUS** (READY, from `AUTONOMOUS.md` and the trifold)
- Score the 4 preloaded pollen before moving
- Collect the 4 wall pollen, then score a second volley from a new spot
- Deploy the flower lever, collect, and score a final volley (the trifold calls this a 2.5-tip autonomous)
- Do it consistently, not just fast: KPI is repeatable autonomous cycles

**TELEOP** (READY)
- Driver never aims by hand: the turret locks the HIVE AprilTag automatically
- Shooter RPM chosen from the measured distance to the tag, so every shot is calibrated
- One driver controls everything (CONFIRM the reason with the team)
- Fast cycles: speed-geared mecanum, passive intake that needs little driving precision

**STRATEGY** (READY, condensed from page 18)
- High priority: launch all 4 pollen in one quick burst, agile drivebase, fast forgiving intake, compact easy-to-build design
- Low priority: shooting nectar. More pollen than nectar is on the field, and supporting both costs throughput and adds transmission and intake complexity

### Design brainstorming (bullets + conclusion). CONFIRM that each was really discussed

| Decision | Options considered | Chosen | Why (from the docs) |
| --- | --- | --- | --- |
| Scoring element | Pollen only vs pollen and nectar | Pollen only | Lower throughput and more complexity for dual shooting; far more pollen on the field (page 18) |
| Launcher | Overshooter (backspin) vs undershooter (frontspin) | Undershooter | Backspin made pollen bounce out; frontspin lowers K_spin and bounce-outs (page 14) |
| Flywheel | Light wheel vs steel flywheel | Steel flywheel, mass at the rim | Higher I = m·r² means less RPM loss and faster recovery between balls |
| Aiming | Driver aims / fixed shot / turret | Limelight AprilTag turret | Removes manual aiming; works while the robot moves |
| Intake | Rigid roller vs motorized articulation vs passive articulation | Passive split-differential | No extra motors, servos or sensors; handles two ball sizes at once |
| Drivetrain | Direct 1:1 vs overdriven | Mecanum, 20T:16T (1.25x) | Faster cross-field cycles; accept 80% of the pushing torque |

**Conclusion:** Every choice was made for one goal: put four pollen in the HIVE as fast and as repeatably as possible,
with the least driver effort. We dropped dual-element shooting early because it cost throughput and complexity.
CAD let us test each idea before cutting parts.

*Add: the real alternatives you considered for drivetrain type (tank, swerve) if you discussed them. The docs do not record this.*

---

## 2. Page 10: Robot Overview (iterations, stats, problem solving)

### Iteration 1
- **Built for:** CONFIRM. Suggested: "A first working 4-pollen shooter, a wide 5-roller intake and a solid-plate chassis, to prove the scoring cycle"
- **Stats:** MEASURE. See the table below.

### Iteration 2 (current)
- **Built for:** improved cycle times and reliability
- **Results:** MEASURE
- **Improvements** (each needs one number):
  - Tighter packaging: weigh both robots, or record the footprint or CG height from CAD
  - Intake refinement (flap covers + lifting flower arm): count jams per 20 intake attempts, V1 vs V2
  - Redesigned chassis layout: time to swap a motor or reach a wire, V1 vs V2

**Measurements to take, as one table** (about 30 minutes of testing):

| Metric | How to measure | V1 | V2 |
| --- | --- | --- | --- |
| Robot weight | Scale | ? | ? |
| Time to load and shoot 4 pollen | Stopwatch, or `shooter_burst_*.csv` | ? | ? |
| Intake jams per 20 attempts | Count | ? | ? |
| Pollen picked up per sweep of the wall | Count over 10 runs | ? | ? |
| Autonomous: 4-pollen volleys scored / 10 runs | Count | ? | ? |

### Problem solving: three real Lesson / Redesign pairs from the repo (READY)

**1. The turret aimed the wrong way in autonomous**
- **Lesson:** Volley 2 missed every run. The team zeroes the turret pointing backward, but the HIVE-memory code assumed encoder 0 meant forward. The HIVE position was mirrored behind the start, so at spot 2 the turret pre-aimed about 134° away from the real HIVE.
- **Redesign:** Added `TURRET_ZERO_DIRECTION_DEG = 180` to convert encoder angles. INIT telemetry now tells the driver to zero backward. Every run also writes a CSV (pose, turret angle, tag offsets, state) so a miss can be traced. (2026-11-13)

**2. Pollen bounced out of the HIVE**
- **Lesson:** The V2 launcher put heavy backspin on pollen, so shots bounced out.
- **Redesign:** V3 undershooter puts frontspin on the ball, plus a steel flywheel for inertia. See page 14.

**3. Wall pollen were missed or the robot stalled at the wall**
- **Lesson:** Driving straight into the wall left some of the four wall pollen uncollected.
- **Redesign:** A "wall shuffle": after Path 2 the robot moves 2 in left, 2.95 in right, back to center, backs off 2.5 in, then pushes 1 in past the stop point, with the intake running. Wall waiting times were then cut 30% (`WALL_TIME_SCALE = 0.7`), and a stall timeout guarantees the robot can never hang there.

*Spares: the flywheel losing speed on ball 4 led to P 90 to 180 and battery-voltage-scaled F (section 6). The bouncing intake bars led to a pivot damper (page 14).*

---

## 3. Page 11: Drivetrain

Everything below is READY and computed from the code.

**Calcs to show (replaces the "graph or calcs" note):**

```
Wheel RPM  = 435 RPM x (20T / 16T)                  = 543.75 RPM
Top speed  = 543.75 / 60 x pi x 0.096 m             = 2.73 m/s  (107.6 in/s)
Overdrive  = 20/16                                  = 1.25x faster than 1:1
Pushing force = 16/20                               = 80% of a 1:1 drivetrain
```

**Theoretical vs measured.** Pedro Foresight tuning measured the real limits:

| | Theoretical | Measured (Pedro tuner) | % |
| --- | --- | --- | --- |
| Forward | 107.6 in/s (2.73 m/s) | 82.3 in/s (2.09 m/s) | 76% |
| Strafe | n/a | 58.8 in/s (1.49 m/s) | 72% of forward |

- Cross-field (144 in) at the measured forward speed is **about 1.75 s**.
- Natural deceleration: 46 in/s² forward, 81 in/s² strafe. These feed Pedro's braking model.
- Suggested chart: bar chart of theoretical vs measured forward vs strafe speed.

**Cycle-maximization number (answers "give me a number"):** use "**1.75 s** to cross the field at top measured speed" or, once measured, your
real full-cycle time (intake to 4 shots). MEASURE that with a stopwatch over 5 cycles.

**Other drivetrain facts (READY):**
- Odometry: goBILDA Pinpoint with 4-bar pods, offsets x = -3.56 in, y = -4.13 in
- Motors: 4 x 13.7:1 Yellow Jacket, 384.5 ticks/rev
- Strafe multiplier 1.1 compensates for mecanum strafe loss in manual drive
- Slow-drive mode: LB gives 40% speed for precise alignment

**Goals box (page 11):** keep as is, but replace "give me a number" with the 1.75 s figure.
Stability evidence: MEASURE (tip test, or "no tip-overs at full-speed stops in N matches").

---

## 4. Page 12: Intake

### Goals (the box currently repeats Drivetrain. Replace with):
- **Touch it, own it:** surface speed at least 1.5x the drivetrain's top speed (4.10 m/s vs 2.73 m/s), so pollen is pulled in even while driving away
- **Both element sizes at once:** pollen and nectar, side by side, with no jams
- **No driver precision needed:** wide mouth, passive compliance, no extra motors or sensors
- **Feed the shooter reliably:** hands pollen to the transmission one at a time

### V1 (CONFIRM from the CAD: solid plates, rigid roller bar)
- Wide multi-roller intake on solid side plates
- Rigid front roller: lost compression on a small ball whenever a large one entered
- No protection against side-by-side balls jamming, and no damping on the bars

### V2 (in use)
- Front roller split into two independent pivoting halves (**split differential passive articulation**)
- Asymmetric roller durometers stagger entry, so no side-by-side jams
- Small holder at each pivot acts as a shock absorber, so the bar settles quickly
- Custom-made 30A gecko wheels and bootkickers, surgical tubing
- Flap covers and a lifting flower arm added in Iteration 2
- Lattice side plates (lighter and more open than V1)

### "Stats, math, notes" (READY, from page 14 and `Robot.java`)

```
V_target = 1.5 x 2.73 m/s = 4.095 m/s
85 mm bootkicker:  RPM = 4.095 / (pi x 0.085) x 60 = 920 RPM
96 mm bootkicker:  RPM = 4.095 / (pi x 0.096) x 60 = 815 RPM
```

- Intake motor runs at full power (`INTAKE_POWER = 1.0`). The transmission runs at 0.6 to index pollen against the closed gate and 1.0 when feeding a volley.
- Chosen: 85 mm bootkickers, which need the higher RPM and give the margin.
- MEASURE: the actual intake RPM (motor RPM x ratio) to show it clears 920, and the pickup success rate V1 vs V2.
- Flag: page 14 says "intake motors" (plural), but the code has one `intake` motor plus a `transmission` motor. Reword if needed.

---

## 5. Page 13: Intake cont. (blank page)

Page 14 already holds the math and challenge table. Use page 13 for the transmission and the gate, which have no page of their own. All READY from the code:

**Intake to shooter handoff**
1. Intake roller pulls pollen in (full power)
2. Transmission indexes pollen back against the **closed gate** (0.6 power)
3. When the shooter is ready, the gate opens, the transmission backs off 0.10 s to unload the gate, then both motors feed at full power

**Gate**
- Servo `torqueservo1` (SWYFT Torque), calibrated endpoints **OPEN 0.41 / CLOSED 0.59**
- A **Gate Calibration** OpMode saves the endpoints on the robot, so a rebuild never needs a code change
- Sequence: gate opens, wait 0.25 s for travel, then feed starts. Pollen are never pushed into a moving gate.

**Flower lever**
- Servo `flower`, **ON 0.75 / OFF 0.415**, toggled by RB, with its own **Flower Calibration** OpMode
- Returns to OFF at INIT and START so it never starts deployed

**Pollen sensor + indicator**
- REV Color Sensor V3 reads proximity (pollen counts when closer than 4.0 cm)
- goBILDA RGB light on the robot: **green = pollen held, red = empty**, so the driver knows without looking

Add images: the CAD of the transmission, the gate, and the two intake versions side by side.

---

## 6. Page 14 fixes and the shooter (page 15 in the PDF)

**Layout fix:** body text runs over the "Design challenges and iterations" heading. Shorten the text or move the table lower.
Page 14 also still has "images like these needed" placeholders.

**Shooter numbers to add** (READY):
- Motor: ServoCity 5000 Series, 28 ticks/rev, 1:1 belt (**verify the part number**)
- Target RPM from camera distance: **1.39 m, 3215 / 1.52 m, 3220 / 1.62 m, 3235**
- Controller: velocity PIDF with **P = 180, I = 0, D = 0, F = 12.71 at 13.3 V** (TeleOp; Auto uses F = 13.83)
- **Battery compensation:** applied F = reference F x 13.3 / battery volts (at 12.8 V, F = 13.21), so shots stay consistent as the battery drains
- Speed readings come in steps of 20 ticks/s = **42.9 RPM**, which is why "ready" needs a stability window rather than one reading
- Ready to shoot: Auto needs within 50 RPM for 300 ms; TeleOp needs within 10 RPM for 150 ms
- Test: four-ball volleys at 3235 RPM, P = 90 / 150 / 180, 3 volleys each in a fixed order so battery drain is fair. MEASURE: paste the 1st-last launch RPM spread and recovery time from `shooter_volleys.csv`. Note that "P=180 looked better on video" is currently the only recorded evidence.
- Flywheel math: add the flywheel's mass and diameter (MEASURE) to show I = ½·m·r² and E = ½·I·ω² at 3215 RPM (ω = 336.7 rad/s)

**Claim to fix on page 18:** "launch all 4 pollen under 0.5 seconds". Nothing in the repo supports it. The tuner doc expects about 1 s, and Auto holds the feed open 2.0 s.
Measure from the burst log and state the real number.

---

## 7. Page 16: Turret / auto-aim ("replace this")

Replacement text for the bullets under the CAD image (READY):
- **Motor:** 1150 RPM Yellow Jacket, 28 x 5.2 = 145 ticks per motor rev
- **Reduction:** 12T to 84T gear (7:1) = **2.82 encoder ticks per degree** of turret
- **Range:** measured physical travel -724 to +563 ticks (about 456°). Software stops 5° inside each end
- **Max speed:** about 986°/s theoretical (1150 RPM x 6 / 7)
- **Camera:** Limelight 3A on the turret, so it turns with the shooter

**Add a "V2 field-anchored aiming" block:**

| | Original | V2 |
| --- | --- | --- |
| Target | Tag bearing from the camera | Remembers the HIVE's **field position** from odometry |
| While strafing or spinning | Only corrects chassis rotation | Stays aimed (feedforward added on top of the 0.22 cap) |
| Tag blinks out | Turret holds, then sweeps | Keeps aiming from odometry |
| Tag choice | Averages every alliance tag | Only the upward CELL's 4-tag cluster |
| Jitter | Raw offsets | 100 ms smoothing, snaps on 3° changes confirmed by 2 frames |
| Shot distance | Single frame | Median of 5 frames, held 1 s |

**Inconsistencies to fix on this page:**
- "−740 to +570 ticks" is the older number. The code measures **-724 / +563**, with software limits near -709 / +548.
- "Forward zero" is true for TeleOp (zero at INIT facing forward). AutoPath zeros the turret facing **backward** (180°).
- "Fires within 3° and 100 RPM" is the original TeleOp. V2 TeleOp uses ±10 RPM for 150 ms and Auto uses ±50 RPM for 300 ms.

---

## 8. New page: TeleOp (Control section)

All READY from `V2.md`. Single-controller layout (gamepad 1):

| Input | Action |
| --- | --- |
| Left stick / Right stick X | Drive + strafe / rotate |
| LB (hold) | Slow drive, 40% |
| RT / LT (hold) | Intake in / out |
| A | Shooter on/off |
| X | Shooting toggle: gate opens and intake feeds, only once the shooter is within ±50 RPM |
| Y | Auto-aim on/off |
| B (hold) | Center turret |
| D-pad left/right | Jog the turret |
| D-pad up/down | RPM trim ±10 (±1 with LB) |
| RB | Flower lever on/off |
| Left stick click | Calibration mode: freeze the table RPM |
| Right stick click | Log distance + RPM to the shot table |

**Driver aids**
- **Auto-aim** locks the HIVE; **Auto RPM** from the distance table; **rumble + LED** when aim, gate and RPM are all ready
- **Pollen light:** green = holding pollen
- **Calibration mode** let us build the shot table at the field: freeze RPM, trim until shots score, log the point to CSV

**Performance work**
- Loop time reduced by switching to manual bulk caching, reading only the distance sensor, and updating telemetry every 100 ms. MEASURE: loop ms before and after (V2 shows loop time on screen).
- Compare old vs new TeleOp protocol exists: 12 shots each from near / mid / far, 3 runs. Paste those make counts here once taken.

*CONFIRM why the team chose one controller; memory says that is how the team drives.*

---

## 9. Page 17: Programming / Autonomous additions (READY)

The page already has the right structure. Add:
- **Route:** 8 Pedro segments, run at 85% of modeled speed. The two segments that drive into balls run slow (35%, 18 in/s)
- **Shot readiness:** gate stays closed until the flywheel is within 50 RPM for 300 ms and the chassis has been still 300 ms. The feed window is then 2.0 s.
- **Fallbacks, so it never hangs:** if the tag is not seen after 2 s, shoot from the table RPM. If RPM is at least 85% of target after 3.5 s, feed anyway. If a volley cannot start within 6 s, skip it and continue.
- **Pre-aim:** the turret learns the HIVE's field position from about 20 sightings while the robot is still, then points there with no tag in view
- **Run log:** every run writes pose, turret state, tag data, shot phase and gate to CSV on the Control Hub
- Page text says P = 180, F = 13.83. That is correct for Auto; TeleOp uses F = 12.71, voltage-scaled.

---

## 10. New page: Onboard processing / electronics (Control section)

READY (hardware names from `Robot.java`):

| Subsystem | Hardware |
| --- | --- |
| Compute | REV Control Hub + Expansion Hub, FTC SDK 12.0.0 |
| Drive | 4 x 13.7:1 motors (384.5 ticks/rev) |
| Intake / transmission | 1 motor each (`intake`, `transmission`) |
| Shooter | 5000-series motor, 1:1, velocity PIDF on the motor encoder |
| Turret | 1150 RPM Yellow Jacket with encoder |
| Gate / flower | 2 positional servos (SWYFT Torque), saved calibration |
| Indicator | goBILDA RGB light on the Expansion Hub |
| Localization | goBILDA Pinpoint, 4-bar odometry pods |
| Vision | Limelight 3A, AprilTag pipeline (alliance tags 30-37 red, 38-45 blue), frames older than 250 ms ignored |
| Pollen sensing | REV Color Sensor V3 (proximity) |

**Software stack:** Pedro Pathing 3.0.2 with Foresight, Ivy 1.1.1 command scheduling, goBILDA Pinpoint driver, AUTO/MANUAL bulk caching.

**Engineering process (strong judging point, READY):**
- Calibration OpModes for gate, flower, turret scale, turret limits and shooter PIDF, so tuning needs no code edits
- Hardware-free regression tests (`TeamCode/tests/`, 10 suites) that run without a robot
- On-robot data logging for volleys, autonomous runs and the shot table

---

## 11. New page: Think (goals, accomplishments, learning plan)

**Accomplishments (READY from the outreach pages):** 14 events, 64 hours, 8 sponsors, about $4000 raised, 2 new members out of 60+ applicants, 2 FLL teams mentored.
**Robot accomplishments:** the split differential passive intake, a 7:1 auto-aim turret, a distance-calibrated shooter, a 4-pollen autonomous.
**Lessons learned (READY):** the three problem-solving stories in section 2, plus "consistency outweighs perfection" and "everyone should get along and communicate" from the trifold.
**Learning plan (CONFIRM with the team):** build the long-range shot table (current table only covers 1.39-1.62 m and clamps at 3235 beyond that); compare old vs new TeleOp over 3 runs; verify turret scale (2.8212 ticks/deg in code vs about 3.02 by hand).
**Competition results:** none in the files. MEASURE/ADD: qualifier results, match scores, awards.

---

## 12. Numbers elsewhere in the portfolio that disagree

- **Outreach master sheet (page 8):** the 13 listed rows add to **2,425** people, not the 2,685+ stated. Hours add up to 64 correctly. The page says 14 events but lists 13, so a row may be missing.
- **Mentors:** page 4 says 4 mentors, page 6 says "5+ mentors".
- **Turret limits and F values:** see sections 6 and 7.
