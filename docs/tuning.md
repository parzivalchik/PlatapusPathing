# Tuning

Every constant, what it means, and what to set it to.

> Doing this for the first time? [BRINGUP.md](../BRINGUP.md) is the ordered
> procedure, on blocks. This page is the reference.

---

## Drivetrain geometry

All measurable or on a datasheet. None of these are tuned by feel.

| Setting | Meaning |
|---|---|
| `trackWidth` | Left-to-right wheel separation, **centre to centre** |
| `wheelBase` | Front-to-back wheel separation, centre to centre |
| `wheelRadius` | Wheel radius, not diameter. A 96 mm wheel is 1.89 in |
| `ticksPerRevolution` | Encoder ticks per revolution **of the output shaft**, after the gearbox. 537.7 for a 312 RPM Yellow Jacket |
| `gearRatio` | Output revolutions per motor revolution. Leave at 1 for a direct mount |
| `maxMotorRpm` | Free speed of the gearbox output, from the datasheet |

If the robot drives the right *shape* but the wrong *scale*, one of
`ticksPerRevolution`, `wheelRadius` or `gearRatio` is wrong. Fix the number — do
not add a fudge factor somewhere else.

### lateralEfficiency

How much of a commanded strafe the chassis actually achieves, in (0, 1].
Mecanum loses more to roller scrub sideways than driving forward.

**Leave it at 1.0 until you have measured it.** Command a known lateral distance,
measure what you got, and set the ratio. A guessed value is worse than none: it
introduces a systematic error you will then tune the PID to hide.

---

## Feedforward

```
motor command = kS·sign(v) + kV·v + kA·a
```

`v` in inches/second, `a` in inches/second². The output is a
**battery-normalised** command in [-1, 1] — the fraction of *nominal* 12 V the
wheel needs, not a raw duty cycle.

**Tune feedforward before PID.** If `kV` is wrong the PID spends the whole path
papering over a systematic error, and no gain will look right.

| Constant | Start at | Then |
|---|---|---|
| `kV` | `1 / maxWheelVelocity` (in/s) | Adjust until `kV × measured_velocity ≈ commanded_power` across several mid-range speeds |
| `kS` | 0 | Raise until the robot *just barely* creeps from rest. Stop there |
| `kA` | 0 | Leave alone until velocity tracking is good. Easiest of the three to make things worse with |

Phase 3's [online tuner](#online-tuning) fits all three for you once the robot
has driven for a while, so this is about getting close enough that the first auto
is safe.

---

## Motion limits

| Setting | Meaning |
|---|---|
| `maxVelocity` | Speed ceiling for the profile, per second |
| `maxAcceleration` | How hard the robot can speed up |
| `maxDeceleration` | How hard it can brake. Defaults to `maxAcceleration` |
| `maxLateralAcceleration` | Cornering grip, in/s². Off by default — see below |

`maxDeceleration` is separate because almost every robot brakes harder than it
accelerates, and profiling both at the lower accel limit wastes real time on
every path.

Start `maxVelocity` at about **a third** of your eventual target. A tuning error
at 15 in/s is a bad demo; the same error at 45 in/s is a broken robot.

Per-segment caps: [`setMaxVelocity()`](paths.md#speed-caps).

### maxLateralAcceleration — cornering grip

**Off by default.** Left off, the profile will plan a hairpin at full speed. The
robot then either slides — at which point the wheels are measuring something the
chassis is not doing, and odometry goes with it — or fails to turn that tightly
and cuts the corner.

Turned on, the profile caps speed by what the corner physically allows:

```
v ≤ sqrt(a_lat / |curvature|)
```

and brakes into a tight corner ahead of time, exactly as it does for a
per-segment cap, so the robot arrives already slow enough.

The ceiling is the friction available, roughly `μ·g` — about 230–390 in/s² for
mecanum on tiles, depending how clean they are. Measure yours: drive a circle of
known radius, raise speed until the wheels break loose, compute `v² / r`, then
take a good margin off it.

It is off by default because enabling it slows some existing paths, and the right
value has to be measured rather than guessed. A straight path is unaffected
either way.

---

## Correction

### translationalPID

Error is in inches, output is inches/second of corrective velocity — so `kP` has
units of 1/second.

Start at `kP = 0.1`, `kI = 0`, `kD = 0.01`. Raise `kP` until the robot corrects
briskly without oscillating.

Reach for `kI` only for a persistent steady-state offset — a constant push from a
partner robot, or a consistently mis-measured `kV`. It is bounded by
`integralLimit` so a pinned robot cannot wind up and lunge when it comes free.

### headingPID

Error is radians, output is radians/second. Start at `kP = 1.5`.

### derivativeFilter

Smoothing in [0, 1), default 0.6. Encoder-derived error is quantised, so raw
`de/dt` at 50 Hz is mostly noise and `kD` amplifies it into audible motor
chatter. Raise toward 0.8 if the robot buzzes; lower toward 0.3 if correction
feels sluggish.

---

## Reactive blending

The hybrid part. Correction authority scales with how far off the path the robot
actually is:

```
authority = 1 + (reactiveAuthority − 1) × clamp(error / reactiveErrorScale, 0, 1)
```

| Setting | Default | Meaning |
|---|---|---|
| `reactiveErrorScale` | 4 in | The error at which correction reaches full authority |
| `reactiveAuthority` | 2.5 | Multiplier at full authority. **1.0 disables blending** |
| `maxCorrectionVelocity` | `maxVelocity` | Ceiling on corrective speed |
| `minConfidenceAuthority` | 0.35 | Floor on how far low pose confidence can scale correction back |

Set `reactiveErrorScale` to roughly the largest error you would still call "on
path". Too small and ordinary tracking error triggers full reactive gain, making
the robot twitchy.

`maxCorrectionVelocity` matters because an uncapped correction at full authority
can demand more speed than the drivetrain has — the wheel commands then saturate
and get scaled down, and it is the *feedforward* that gets squeezed out.

Put `getCorrectionAuthority()` on telemetry while tuning this. It is the one
number that says which regime the follower is in: 1.0 means tracking fine, rising
means fighting to get back. If it sits high all the time, `reactiveErrorScale` is
too small for how well your robot actually tracks.

---

## Arrival

| Setting | Default | Meaning |
|---|---|---|
| `positionTolerance` | 1 in | How close counts as arrived |
| `headingTolerance` | 2° | How close counts as pointed the right way |
| `settleTimeout` | 0.5 s | How long to keep correcting after the profile ends before giving up |

`settleTimeout` exists because a robot pinned an inch short of its target would
otherwise hold the auto hostage for the rest of the match. The follower gives up
and moves on, and reports the failure through `isAtTarget()` — which is why
`!isBusy()` is not the same as success.

---

## Online tuning

`kS`, `kV` and `kA` fit themselves while the robot drives. No tuning OpMode,
nothing to run back and forth at fixed powers.

```java
TunableFeedforward feedforward = new TunableFeedforward(new FeedforwardGains(0.05, 0.017, 0.0));
FtcTuningStore store = new FtcTuningStore();

TuningSession tuning = TuningSession.builder(drivetrain, voltage, clock, feedforward)
        .forgettingFactor(0.99)
        .store(store)
        .build();

tuning.restore();   // at init: pick up where the last run left off

PathFollower follower =
        new PathFollower(drivetrain, localizer, constants, clock, feedforward);

// each loop, after follower.update():
tuning.update(follower.getLastPowers());

// at the end of the OpMode:
tuning.persist();
```

| Setting | Default | Meaning |
|---|---|---|
| `forgettingFactor` | 0.99 | How fast old samples decay. 0.98–0.995 is the usable band — roughly a 200 to 2000 sample memory |
| `minVelocity` | 2 in/s | Below this, samples are discarded |
| `maxCommand` | 0.95 | At or above this, samples are discarded |
| `minSamplesBeforeProposing` | 400 | Samples before a fit is offered. ~2 s of driving at 4 wheels × 50 Hz |
| `minConfidenceBeforeProposing` | 0.5 | How well-determined the fit must be first |

Lower `forgettingFactor` tracks a changing robot faster but gets twitchy; higher
averages over more history and is stable but slow.

Two classes of sample are **dropped**, not down-weighted. Near-zero velocity,
where `sign(v)` flips on noise and carries no information about static friction.
And saturated commands, where the motor is not obeying the model at all — fitting
to those teaches the model that speed is cheap, biasing `kV` down and making
every subsequent auto undershoot.

### Gains never change mid-path

`TunableFeedforward` holds an *active* set and a *pending* one. The tuner
proposes at any time; the follower commits only inside `followPath()`. Swapping
the model underneath a running profile makes the response discontinuous at an
arbitrary moment and the resulting miss impossible to reproduce.

Proposals are screened: implausible fits are refused, and a fit within 2% of what
is already active is not worth the churn.

`FtcTuningStore` writes to `/sdcard/FIRST/platapuspathing/feedforward.properties` —
three readable lines, survives an app update, deletable over ADB. A corrupt or
implausible file is refused rather than loaded, because driving on a `kV` of NaN
is not recoverable.

---

## Stall detection

| Setting | Default | Meaning |
|---|---|---|
| `stallCurrentAmps` | 9.2 | Motor stall current at 12 V, from the datasheet. Winding resistance is derived from it |
| `currentRatioThreshold` | 2.0 | Multiple of predicted current that counts |
| `minAbsoluteExcessAmps` | 1.5 | Amps above prediction also required |
| `debounceSeconds` | 0.2 | How long the condition must hold continuously |

`minAbsoluteExcessAmps` guards the low end: when the model predicts 0.2 A, a
perfectly healthy 0.5 A reading is 2.5× prediction and means nothing.

If healthy driving already reads 3× over, your `stallCurrentAmps` is wrong for
your motor.

The debounce requires the condition to hold **continuously** — alternating spikes
that sum past the window do not fire.

See [Localization](localization.md) and [Architecture](architecture.md#stall-detection)
for why the prediction uses *commanded* rather than measured velocity.

---

## Quick failure table

| Symptom | Look at |
|---|---|
| Spins instead of driving forward | Motor reversal |
| Strafes the wrong way | Roller pattern; swap front/back pairs |
| Right shape, wrong scale | `ticksPerRevolution`, `wheelRadius`, `gearRatio` |
| Tracks fine slow, drifts off fast | `kV` too low — wheels saturating |
| Never arrives, times out | `kS` too low, or `positionTolerance` too tight |
| Oscillates around the path | `translationalPID` kP too high |
| Corrects violently on small errors | `reactiveErrorScale` too small |
| Motors buzz audibly | `derivativeFilter` too low, or `kD` too high |
| Lunges after being released | `integralLimit` too high |
