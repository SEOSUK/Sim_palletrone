# Modeled 500 g payload-position map

## Purpose and scope

This pilot maps Lissajous mission performance versus modeled payload x-y
position with the frozen 0.5 kg v5 plant/controller/MOCE parameters. Repository
search found no CAD-derived mounting plate, rail, fixture, or allowable x-y
boundary. The results are therefore a **modeled 500 g payload-position map**, not
a physically mountable or safe envelope.

Setup-2 is confirmed as 0.5 kg at FLU `[0.31,0.31,0.56] m`. Grid z remains
0.56 m. The 25 diagnostic points use x,y =
`0.31 * [-1.25,-0.625,0,0.625,1.25]`, or
`[-0.3875,-0.19375,0,0.19375,0.3875] m`. Setup-2 is marked separately in plots.

## Mission-only sequence and equivalence

The plant starts at the configured hover `[0,0,0.7] m`, nominal attitude, and
MuJoCo's zero initial velocity/rate. Controller filters and integrators reset
normally from that state. After 0.5 s initialization, MOCE is enabled for 15 s
hover; Z adaptation and the unchanged 60 s Lissajous reference then start. This
preserves the original MOCE-on pre-trajectory adaptation duration without the
out-of-scope ascent. Gains and model parameters are unchanged.

For Setup-2 and seeds 7000-7014, mission-only versus original automated
eligible-phase medians were:

| metric | original | mission-only |
|---|---:|---:|
| position RMS (m) | 0.0760 | 0.0753 |
| attitude RMS (rad) | 0.0197 | 0.0191 |
| peak attitude (rad) | 0.0578 | 0.0528 |
| max motor utilization | 0.415 | 0.413 |
| max servo angle (rad) | 0.447 | 0.475 |
| max servo rate (rad/s) | 3.06 | 3.20 |
| realized torque residual RMS (Nm) | 0.337 | 0.331 |
| CoM error RMS (m) | 0.0230 | 0.0251 |
| normalized allocation sigma min | 0.845 | 0.836 |
| auxiliary mean DOB norm | 0.194 | 0.193 |

The mission metrics are sufficiently consistent for a diagnostic map. Servo
angle/rate and CoM-error differences are retained as limitations; this is not a
claim that the initial histories are statistically identical.

## CRN, classification, and results

Five replicates at every point reuse measurement, torque-OU, and force-OU seeds
`7000 + 3*j + {0,1,2}`. All 125 trials initialized, survived stabilization,
were Lissajous-eligible, and completed the mission. There were zero
initialization failures, stabilization failures, and mission losses. No tracking
threshold was used.

Across point medians:

| metric | minimum | maximum |
|---|---:|---:|
| position RMS (m) | 0.0750 | 0.0757 |
| attitude RMS (rad) | 0.0174 | 0.0203 |
| peak attitude (rad) | 0.0461 | 0.0557 |
| max motor utilization | 0.292 | 0.451 |
| max servo angle (rad) | 0.325 | 0.641 |
| max servo rate (rad/s) | 2.86 | 5.82 |
| realized torque residual RMS (Nm) | 0.327 | 0.335 |
| CoM error RMS (m) | 0.0246 | 0.0255 |
| normalized allocation sigma min | 0.807 | 0.944 |

Position tracking changes by less than 1%, whereas motor utilization changes
55%, servo angle nearly doubles, servo rate doubles, and normalized allocation
margin falls about 15%. Thus control/actuator margin changes well before
position RMS. Servo demand is the most position-sensitive measured margin.

## Directionality

The equal-radius four corners show similar attitude RMS (0.0195-0.0203), motor
utilization (0.439-0.451), and allocation sigma minimum (0.807-0.819), but clear
servo-direction effects: maximum servo angle ranges 0.546-0.641 rad and maximum
servo rate 3.16-5.82 rad/s. This supports directional dependence in servo
demand, consistent with the rotor/tilt-axis arrangement, but does not support a
large directional position-tracking or mission-continuation effect in this
pilot.

Realized torque residual and CoM estimation error vary only 2-4%; neither is the
dominant spatial signal. DOB metrics are retained only for future in-sample
Setup-2 flight consistency checks. Hardware realized thrust is not measured, so
realized-wrench residual should not be directly compared with flight logs.

## Recommendation

Choose **Option D first**: obtain the actual physical mounting geometry and
restrict the modeled grid to mountable positions. Then use **Option C** to refine
the mountable portions corresponding to high servo angle/rate, especially the
negative-x/negative-y angle-demand region and mixed-sign high-rate corners.
The clean CRN maps do not presently justify increasing every point to 30-50
replicates, and no mission-loss boundary was found.
