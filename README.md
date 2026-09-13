# VerniteYaku

A path-following library for FIRST Tech Challenge robots.

It takes the parts of Road Runner and Pedro Pathing that each do well — Road
Runner's time-consistent motion profiling, Pedro's readable path-chain API and
aggressive on-path correction — and builds toward the things neither offers yet:
fused localization rather than a single sensor, online feedforward auto-tuning,
and per-motor stall detection.

It is a **library**, not a team's robot code. Nothing here is specific to one
chassis, one hub layout, or one season.

Not yet run on a robot — see [BRINGUP.md](BRINGUP.md) before you try.

---

## Quickstart

Two feet forward, then a curve away to the left, turning to face left as it goes:

```java
MecanumDrivetrain drivetrain = MecanumDrivetrain.builder(DistanceUnit.INCH)
        .motors(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight")
        .reverse(true, false, true, false)
        .trackWidth(15.0).wheelBase(13.0).wheelRadius(1.89)
        .ticksPerRevolution(537.7).maxMotorRpm(312)
        .build();

OdometryComputerLocalizer localizer = OdometryComputerLocalizer.builder(tracker)
        .startPose(new Pose2d(-60, -36, 0))     // where the robot is placed
        .build();

FollowerConstants constants = FollowerConstants.builder(DistanceUnit.INCH)
        .maxVelocity(40).maxAcceleration(50).maxDeceleration(60)
        .kV(0.017).kS(0.05)
        .translationalPID(0.15, 0.0, 0.01)
        .headingPID(1.5, 0.0, 0.05)
        .build();

PathFollower follower = new PathFollower(drivetrain, localizer, constants);

PathChain chain = new PathBuilder()
        .addPath(new BezierLine(new Point(-60, -36), new Point(-36, -36)))
        .setLinearHeadingInterpolation(0, Math.toRadians(90))
        .addPath(new BezierCurve(new Point(-36, -36), new Point(-24, -24), new Point(-24, -6)))
        .setConstantHeadingInterpolation(Math.toRadians(90))
        .build();

waitForStart();

follower.followPath(chain);
while (opModeIsActive() && follower.isBusy()) {
    follower.update();
    telemetry.addData("pose", follower.getPose());
    telemetry.update();
}
```

The full version, with telemetry and comments, is in
[`ExampleAutoOpMode`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ExampleAutoOpMode.java).
[`ExampleHoldPositionOpMode`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ExampleHoldPositionOpMode.java)
shows holding a pose while a mechanism runs, and
[`ExampleTunedAutoOpMode`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ExampleTunedAutoOpMode.java)
adds online tuning and stall detection.

## Documentation

| Guide | For |
|---|---|
| [Getting started](docs/getting-started.md) | Adding the library and running your first path |
| [Paths](docs/paths.md) | Curves, chains, headings, speed caps |
| [Tuning](docs/tuning.md) | Every constant and what to set it to |
| [Localization](docs/localization.md) | Odometry, the EKF, vision |
| [Commands](docs/commands.md) | Optional sequencing, and how to skip it |
| [Architecture](docs/architecture.md) | How it fits together, for contributors |
| [Migrating](docs/migrating.md) | Coming from Pedro Pathing or Road Runner |
| [Bring-up](BRINGUP.md) | First time on real hardware |

API reference: `./gradlew :core:javadoc` &rarr; `core/build/docs/javadoc/index.html`

---

## Conventions

**Coordinate frame — the field.** Origin at the centre of the field, +x and +y in
the floor plane, heading CCW-positive radians from +x. The field is 144 inches
square, so coordinates run −72 to +72.

`new Point(-36, -36)` is a fixed spot, and it means the same thing whichever tile
the robot starts on. This is the convention the FTC SDK's AprilTag support uses,
which is what lets a tag observation drop straight into the pose filter.

The flip side: absolute coordinates cannot be inferred from encoders, so the
localizer has to be told where the robot is placed.

```java
OdometryComputerLocalizer localizer = OdometryComputerLocalizer.builder(tracker)
        .startPose(new Pose2d(-60, -36, 0))
        .build();
```

Get that wrong and every path is offset by the same amount — the robot drives the
right shape in the wrong place. It is the first thing to check when an auto is
uniformly off.

`FieldCoordinates` holds the constants and helpers: field and tile size,
`contains()` to check a plan fits inside the walls, `rotated180()` to mirror a
plan to the other alliance.

**One axis question to settle on a real field.** Origin and handedness are fixed;
*which physical wall +x points at* is a choice that must match your AprilTag
layout and IMU zero. No season-specific mapping is baked in. Check it once —
place the robot at a known spot, read `getPose()`, confirm the signs — and if +x
points the other way, `FieldCoordinates.rotated180()` the whole plan rather than
negating coordinates one at a time.

**Units — your choice, converted once.** Everything is stored internally in
inches, but you never have to work in them. Every builder takes a
`DistanceUnit`, and values are converted the moment they enter:

```java
FollowerConstants.builder(DistanceUnit.CM).maxVelocity(100)  // 100 cm/s
Point.of(60.96, 0, DistanceUnit.CM)                          // 24 inches
follower.getPose().getX(DistanceUnit.CM)                     // read back in cm
```

`INCH`, `CM`, `MM` and `METER` are all available. Because conversion happens at
the boundary, a centimetre value can never leak into a computation expecting
inches.

**Angles are always radians**, everywhere, with no unit switch. Use
`Math.toRadians()` at the call site.

---

## Layout

```
core/       Pure Java. No FTC SDK, no Android. All the math and control lives here.
ftc/        Android library. The thin hardware wrappers — four DcMotorEx and an IMU.
TeamCode/   Sample module with one worked OpMode.
tools/      The alliance collision planner (browser tool, no build step).
```

The split between `core` and `ftc` is what makes `./gradlew :core:test` run on
any laptop with a JDK, with no Android SDK, no emulator, and no robot. Every
Bezier, every kinematics conversion, and the follower itself are exercised that
way — 274 tests, all headless.

`:ftc` and `:TeamCode` are only included in the build when an Android SDK is
actually present, so cloning this repo and running the tests works on a machine
that has never had Android Studio installed.

### Layers

Each depends only on the one below it:

| Layer | Package | What it does |
|---|---|---|
| Command | `command` | Optional sequencing. Nothing depends on it |
| Follower | `follower` | Turns a path and a pose into wheel commands |
| Control | `control` | Motion profile, PID, feedforward constants |
| Paths | `paths` | Bezier curves, chains, heading interpolation |
| Localization | `localization` | Where the robot is. An odometry computer, drive encoders, or the EKF |
| Kinematics | `kinematics` | Chassis motion ↔ wheel motion |
| Drive | `drive` | The four-method hardware seam |
| Tuning | `tuning` | Online feedforward fitting, stall detection |

The follower talks to `Drivetrain` and `Kinematics`, never to a mecanum
directly — tank and swerve slot in without the follower changing.

---

## How the follower works

Each `update()`:

1. The **motion profile** says where the robot should be by now, as a distance
   along the chain. This runs on a clock, not on measured progress, so a path
   takes the same time every run.
2. That distance resolves to a **point, tangent and desired heading** on the
   chain.
3. **Feedforward** drives along the tangent at the profiled speed, via
   `kS·sign(v) + kV·v + kA·a` applied per wheel.
4. **PID** adds a correction proportional to how far the robot actually is from
   that setpoint.
5. The combined chassis velocity goes through the drivetrain's kinematics, and
   the resulting wheel commands are scaled — not clipped — into motor powers.

Scaling rather than clipping matters: clipping one saturated wheel changes the
ratio between the four, which changes the direction the robot actually travels.
That is the usual reason a robot tracks a path fine slowly and drifts off it
fast.

### Per-segment speed limits

`setMaxVelocity()` caps one segment without slowing the whole run — a careful
approach into a scoring position, then a fast sprint back.

That needs more than a trapezoid, so the profile is solved numerically with a
forward-backward sweep: a forward pass enforcing "you cannot speed up faster than
the robot can", then a backward pass enforcing "you must already be braking
before the slow bit". The backward pass is the part a naive per-segment profile
gets wrong — it arrives at the slow segment still going fast and then brakes
harder than the robot physically can.

With no caps set, the ceiling is constant and the result is the same trapezoid as
before; the tests assert the numerical solver reproduces the closed form.

### The reactive term

Step 4 is not a plain PID. The correction is multiplied by an *authority* that
scales with how far off the path the robot actually is:

```
authority = 1 + (reactiveAuthority - 1) * clamp(error / reactiveErrorScale, 0, 1)
```

While the robot is tracking well, authority sits at 1 and the follower behaves
like Road Runner: smooth, time-consistent, gentle. Knock it off the path and
authority ramps toward `reactiveAuthority`, and it starts behaving like Pedro:
correcting hard and immediately. Getting both out of one follower, without
switching between two of them, is the point of the whole design.

Blending on error magnitude rather than switching at a threshold is deliberate —
a hard switch puts a discontinuity in commanded velocity exactly where the robot
is already struggling, and a robot hovering near the threshold chatters between
two control regimes.

Set `reactiveAuthority(1.0)` to disable it and get a pure profile follower back.

With a fusing localizer, authority is also scaled by pose confidence, floored at
`minConfidenceAuthority`. Correcting aggressively toward a pose you do not
believe is how a robot ends up chasing its own estimation error across the field.

---

## Localization

Two implementations, same interface — the follower cannot tell them apart.

**`DriveEncoderLocalizer`** is dead reckoning from the drive encoders, with the
IMU optionally overriding wheel-derived heading. No extra hardware, and the least
accurate option: mecanum wheels scrub sideways and the encoders cannot see it.

**`FusedLocalizer`** runs an extended Kalman filter over `[x, y, heading]`.
Odometry is the prediction step; the IMU — and later AprilTags — enter as
*measurement updates* weighted by their own variance.

That distinction is the whole point. The common FTC approach of calling
`resetPose()` when a tag comes into view discards a good odometry estimate in
favour of a single frame's observation, and teleports the robot mid-path when
that observation is marginal. Here a measurement moves the estimate in proportion
to how much it deserves to:

| Observation variance | Gap closed over 60 fixes |
|---|---|
| 0.25 in² (vague) | 67% |
| 0.05 in² | 84% |
| 0.01 in² (tight) | 96% |

The filter also knows that lateral odometry is worse than forward odometry, so
process noise is built in the robot's frame and rotated into the world. A heading
measurement therefore corrects *position* too, through the correlations the
covariance carries — information a `resetPose()` override throws away.

`getConfidence()` reports a bounded summary of positional uncertainty, which is
what feeds the follower's authority scaling.

### Vision

`VisionPoseSource` is a **stub**. The interface and the `FusedLocalizer` plumbing
that consumes it are finished and tested against scripted observations; the
AprilTag half — camera calibration, tag-field layout, turning a detection into a
field pose — is not built. It exists now because the shape of that interface
constrains the filter's design, and an observation must carry its own variance for
any of the above to work.

Field coordinates make the remaining work markedly easier than the old
start-relative frame did: tag positions are fixed and published, so nothing in
the pipeline needs to know where the robot began.

---

## Building

```bash
./gradlew :core:test
```

Runs the whole test suite. Needs only a JDK 17+.

```bash
./gradlew build
```

Builds everything, including the Android modules — needs an Android SDK with
API 30, either via `ANDROID_HOME` or `local.properties`.

### Deploying to a robot

`TeamCode` here is a sample library module, not a deployable app: it proves the
code compiles against the real SDK, but the robot controller app itself is not
vendored into this repo. To actually run this on a robot, either copy
`ExampleAutoOpMode` into your own `FtcRobotController` project's TeamCode and
depend on `:core` and `:ftc`, or add those two modules to that project's
`settings.gradle`.

---

## Tuning

Start here, then adjust on the robot:

| Constant | Start at | Then |
|---|---|---|
| `kV` | `1 / maxWheelVelocity` (in/s) | Raise until the robot holds profile speed |
| `kS` | 0 | Raise until the robot just barely creeps from rest |
| `kA` | 0 | Leave alone until velocity tracking is good |
| `translationalPID` kP | 0.1 | Raise until it corrects briskly without oscillating |
| `headingPID` kP | 1.5 | Same |
| `reactiveErrorScale` | 4 in | The largest error you call "still on path" |
| `reactiveAuthority` | 2.5 | Raise for harder recovery; 1.0 disables it |

Tune feedforward **before** PID, and do the whole thing on blocks first —
[BRINGUP.md](BRINGUP.md) is the step-by-step, including the failure table for
when the robot does something baffling.

Put `getCorrectionAuthority()` on telemetry while tuning the blend — it is the
one number that says which regime the follower is currently in.

Tune feedforward *before* PID. If `kV` is wrong, the PID spends the whole path
papering over a systematic error and no gain will look right.

Phase 3 will fit `kS`, `kV` and `kA` online by recursive least squares, so this
table becomes a starting point rather than a chore.

---

## Roadmap

- **Phase 1** — paths, kinematics, profile + feedforward + PID follower. *Done.*
- **Phase 2** — EKF pose fusion (encoders + IMU, vision stubbed); error-scaled
  reactive correction with a tunable blend weight. *Done.*
- **Phase 3** — pluggable command layer; RLS auto-tuning of `kS`/`kV`/`kA`
  persisted between OpMode runs; per-motor stall detection against the
  feedforward model's predicted current, debounced and surfaced as a callback.
  *Done.*
- **Alliance planner** — *done*, see below.

---

## Auto-tuning

`kS`, `kV` and `kA` fit themselves while the robot drives. There is no tuning
OpMode and nothing to run back and forth at fixed powers.

The feedforward model is linear in its parameters, so every control loop is one
row of a least-squares problem:

```
u = kS·sign(v) + kV·v + kA·a  =  θ·x,   θ = [kS, kV, kA],  x = [sign(v), v, a]
```

Recursive least squares with an exponential forgetting factor (0.98–0.995) keeps
the fit current, so it tracks a robot that changes mid-competition — a belt
tightening, a wheel picking up grit.

**Samples are normalised by battery voltage**, `duty × V_battery / 12`, not fed
in as raw duty cycle. A duty of 0.5 is a different applied voltage at 13 V than
at 11 V, so a model fitted against raw duty silently re-fits itself as the
battery drains — which is exactly when autos start missing.

Two classes of sample are dropped rather than down-weighted: near-zero velocity,
where `sign(v)` flips on noise and carries no information about static friction;
and saturated commands, where the motor is not obeying the model at all and
fitting to it teaches that speed is cheap, biasing `kV` down.

### Gains never change mid-path

`TunableFeedforward` holds an *active* set and a *pending* one. The tuner
proposes at any time; the follower commits only inside `followPath()`. Swapping
the model underneath a running profile makes the response discontinuous at an
arbitrary moment and the resulting miss impossible to reproduce, so the rule is
enforced by the structure rather than by remembering to obey it.

Proposals are also screened: implausible fits are refused, and a fit within 2% of
what is already active is not worth the churn of adopting.

### Persistence

`FtcTuningStore` writes to `/sdcard/FIRST/verniteyaku/feedforward.properties` —
three human-readable lines, survives an app update, deletable over ADB when you
want to start over. Call `restore()` at init and `persist()` at the end, and the
second match starts from the first match's answer. A corrupt or implausible file
is refused rather than loaded, because driving on a `kV` of NaN is not
recoverable.

---

## Stall detection

Per motor, compared against what the tuned model says the motion should cost.

A fixed "over 6 amps means stalled" threshold is wrong in both directions: it
fires every time a healthy robot accelerates hard, and misses a genuinely wedged
wheel at low commanded power. Current alone doesn't say whether a motor is in
trouble; current *versus what this motion should cost* does.

```
I_expected = (V_applied − V_backEmf) / R
```

`V_applied` is duty × battery voltage. The back EMF is exactly the feedforward's
`kV·v` term — the voltage fraction needed to sustain that speed. And `R` comes
from the motor's datasheet stall current, since a stalled motor has no back EMF:
`R = 12 / I_stall`.

**The back-EMF term uses the velocity the follower *commanded*, not the one the
encoder reports.** This is the whole trick. Predicting from measured velocity is
self-fulfilling — a wedged wheel reads zero speed, so the model predicts zero
back EMF and therefore a large current, which is precisely the large current the
motor is really drawing. Measured and predicted agree perfectly and nothing ever
looks wrong. Predicting from commanded velocity asks the useful question instead:
*if this wheel were turning as fast as we told it to, what would it cost?*

For a robot cruising at 30 in/s, that's a predicted 0.74 A against 5.3 A if the
wheel is jammed — a ratio of 7. Debounced over 150–300 ms so a seam crossing
isn't a stall.

The library then does **nothing**. Aborting, retrying, backing off, cutting power
— all reasonable, all game- and mechanism-specific. `StallListener` reports and
your OpMode decides.

---

## Command layer

Entirely optional. `Follower` doesn't know it exists, and if you already use
FTCLib or Road Runner actions you can ignore this package and drive the follower
from those — it's three methods.

There is no scheduler singleton, no subsystem model, no requirement declarations,
and no exclusive-resource arbitration. What it gives you is composition:

```java
CommandRunner runner = new CommandRunner(Commands.sequence(
        new FollowPathCommand(follower, toBasket),
        Commands.run(arm::raise),
        Commands.waitSeconds(0.4, clock),
        new FollowPathCommand(follower, toPark)));

while (opModeIsActive() && !runner.isFinished()) {
    runner.run();
}
```

`sequence`, `parallel`, `race`, `deadline`, `waitSeconds`, `waitUntil`, `either`,
plus fluent `andThen` / `alongWith` / `raceWith` / `withTimeout`. An interrupted
`FollowPathCommand` stops the drivetrain; a command that spun up a shooter has to
switch it off in its own `end`, since the runner has no way to know what needs
undoing.

See [`ExampleTunedAutoOpMode`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ExampleTunedAutoOpMode.java)
for all three Phase 3 pieces working together.

---

## Alliance planner

`tools/alliance-planner/index.html` — open it in a browser. No build step, no
server, no dependency on the Java library.

**Plan for one robot or two.** Two robots checks both alliance partners' autos
against each other; one robot is just a path editor for your own. Robot 2 stays
in memory while hidden, so switching back does not lose its path.

### What it checks

Collision testing is the separating axis theorem on rotated rectangles, not a
bounding-circle approximation. The difference is not academic — for two 18"
robots, a circle test reports a collision at every centre distance under 25.5",
including cases with 7" of genuine clearance:

| Centres apart | Bounding circle | SAT | True clearance |
|---|---|---|---|
| 19" | COLLIDE | clear | 1" |
| 22" | COLLIDE | clear | 4" |
| 25" | COLLIDE | clear | 7" |

The same exact test runs against **obstacles** — named, coloured, rotatable
rectangles for goals, trusses, or a partner's parked robot. The verdict names
whatever got hit and when.

### Why the timeline is trustworthy

It runs the same trapezoidal profile and the same forward-backward velocity
solve the follower does, including per-segment speed caps and the same
curvature-derived cornering limit. So the timeline
reflects when each robot is actually *somewhere*, not just where its path goes.

Per-robot **start delays** let you test "wait two seconds, then go" without
touching either path. **Waits** park a robot mid-path — and because a wait is a
real stop, the chain splits there and the robot brakes to rest and accelerates
again. A 2 s wait typically costs nearer 2.7 s; the timeline shows the true
figure rather than the one you asked for.

A robot that finishes its path stays parked there and still counts, which is
usually the collision people miss.

### Coordinates match the library

Everything is field coordinates, the same frame `PathChain` and `Pose2d` use, so
the numbers on the canvas are the numbers you paste. A robot's start position
*is* its path's first point, so the two cannot drift apart and describe a robot
placed somewhere its own auto does not begin.

The Java export emits ready-to-paste `pathBuilder()` code plus a commented
`.startPose(...)` line for the localizer — that belongs on the localizer, not the
path.

`PlannerExportTest` compiles a verbatim copy of that export, so a renamed
builder method breaks the build rather than breaking someone's auto at a
competition.

### The rest

- **Safety margin, 2" by default.** Footprints are inflated before testing. A
  plan that only just clears at zero margin has no slack for the odometry drift a
  real robot accumulates over a match.
- **Obstacles are tagged** neutral, red, blue or hazard, so a field with several
  of them and two robots stays readable.
- **Heading arrow** per robot, toggleable — the fastest way to see whether a
  robot is facing where it is going or holding a fixed heading while it
  translates.
- **Field backdrop** — the 2026–27 BIOBUZZ field is built in and shows on a
  fresh load (`tools/alliance-planner/assets/field-biobuzz.png`, a top-down
  render of the official field; red alliance −Y, blue +Y, audience +X — the
  Competition Manual's §9.5 view, red on the audience's left, rotated a quarter
  turn). Drop in another official image and
  it stretches over the 144" square instead. No game field is drawn from
  memory: one that is subtly wrong is worse than none, because you would plan
  against fiction.
- **Exports** — Java code, plan JSON, a sampled points array, or the field as a
  PNG. The code view is full-height and says which robots it covers.
- **Save / open** — plans are real `.json` files you can commit next to your
  auto. Drop one on the canvas to open it.
- **Undo / redo** across every edit, plus a hard reset for a plan that has got
  somewhere you would rather not unpick a step at a time.
- **Named plan variants** — New / Duplicate / Remove, so candidate routes sit
  side by side instead of overwriting one another. Every plan is kept in the
  saved `.json`.
- **Cornering limit** matching `FollowerConstants.maxLateralAcceleration`, so a
  hairpin is planned at a speed the robot could actually hold.
- **Ruler and protractor** (`M`) — drag to read a distance and a field heading
  without dropping a throwaway path point just to see a coordinate. The angle is
  a field heading, so it goes straight into `setConstantHeadingInterpolation`.
- **Robot marker images** — drop a picture on either robot to tell them apart at
  a glance. Painted into the footprint, so the picture is exactly the box that
  gets collision-tested rather than a decoration at a different size.
- **Keyboard shortcuts** behind the `?` button, or press `?`.

Numbers in the planner are **planner-only**: motion limits and robot dimensions
describe the simulation and do not reach your robot's code. Copy them into
`FollowerConstants` and `MecanumDrivetrain` yourself, or the plan and the robot
will quietly disagree. The panel says so where you enter them.

## Licence

MIT. See [LICENSE](LICENSE).

`TeamCode/` vendors one third-party file: goBILDA's `GoBildaPinpointDriver.java`
(MIT, © Base 10 Assets LLC), with its notice intact. It is not on Maven Central,
so vendoring it is the only way the Pinpoint sample compiles. `:core` and `:ftc`
do not depend on it. To update, re-copy from
[FtcRobotController-Add-Pinpoint](https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint)
(branch `goBILDA-Odometry-Driver`); `PinpointOdometryComputer` is the only thing
that would break.
