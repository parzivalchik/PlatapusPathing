# Architecture

For contributors, and for anyone deciding whether the design will survive contact
with their robot.

---

## Module split

```
core/       Pure Java. No FTC SDK, no Android. All the math and control.
ftc/        Android library. Four DcMotorEx, an IMU, a voltage sensor, a file path.
TeamCode/   Sample module with two worked OpModes.
tools/      The alliance planner. Browser tool, no build step, no dependency on core.
```

The split is not cosmetic. `MecanumDrivetrain` imports `DcMotorEx`, which is an
Android artifact — a single module would force the whole library, Beziers
included, into an Android library needing the SDK and Robolectric to test.

Instead `:core` is a plain `java-library`. `./gradlew :core:test` runs on any
laptop with a JDK 17: no Android SDK, no emulator, no robot. `settings.gradle`
only includes the Android modules when an SDK is actually present, so cloning and
testing works on a machine that has never had Android Studio.

Everything hardware-facing is about 300 lines in `:ftc`. That is the entire
surface between this library and the FTC SDK.

---

## Layers

Each depends only on the one below it.

| Layer | Package | Depends on |
|---|---|---|
| Command | `command` | Follower. **Nothing depends on it** |
| Follower | `follower` | Control, Paths, Localization, Drive |
| Control | `control` | Paths (for velocity limits), Geometry |
| Paths | `paths` | Geometry |
| Localization | `localization` | Kinematics, Geometry, Math |
| Kinematics | `kinematics` | Geometry |
| Drive | `drive` | Kinematics |
| Tuning | `tuning` | Drive, Math |
| Geometry / Math / Units | — | Nothing |

The follower talks to `Drivetrain` and `Kinematics`, never to a mecanum directly.
Tank and swerve slot in without the follower changing.

---

## The two-interface drivetrain

A single interface cannot be both pure-math-testable and the thing that writes
motor powers, so there are two:

- **`Kinematics`** — pure math, no SDK. `toWheelVelocities(ChassisSpeeds)` and
  `toChassisSpeeds(double[])`. Fully unit-tested.
- **`Drivetrain`** — the hardware seam. Four methods: `getKinematics()`,
  `setWheelPowers()`, `getWheelPositions()`, `getWheelVelocities()`.

The follower gets its math through `getKinematics()`, so it can be tested against
a fake `Drivetrain` with no hardware. `SimulatedRobot` in the test sources
implements both `Drivetrain` and `Localizer`, closing the loop.

### Scale, don't clip

When any wheel exceeds top speed the whole set is scaled down together. Clipping
one saturated wheel changes the ratio between the four, which changes the
direction the robot actually travels — the usual reason a robot tracks a path
fine slowly and drifts off it at speed.

---

## Paths

`Path` is pure geometry: point, derivative, second derivative, arc length. No
heading, no velocity, no time. Heading is layered on by a `HeadingInterpolator`
inside a `PathSegment`, so the same curve can be driven nose-first or at a fixed
angle without touching the curve.

Bezier evaluation is de Casteljau. The derivative of a degree-n Bezier is a
degree-(n-1) Bezier over control points `n·(P[i+1] − P[i])`, so derivatives are
exact rather than numerical.

**Arc length is tabulated at construction** — 200 cumulative samples, giving
constant-time `arcLengthAt` and `tAtArcLength` afterwards. This matters because
the follower calls them every control loop, and because `t` is not proportional
to distance: stepping `t` directly gives uneven ground spacing.

`getClosestT` does a coarse global scan for a bracket, then prefers the
neighbourhood of the caller's guess when it is competitive, then golden-section
refines. The guess is what breaks the tie correctly on a path that crosses
itself.

---

## Motion profiling

`MotionProfile` is the closed-form trapezoid: one speed limit, solved
analytically. Kept as the reference implementation.

`ConstrainedProfile` is what the follower actually uses. It handles a ceiling
that varies along the path, solved numerically with a forward-backward sweep:

1. Sample the path at a fixed arc-length step, reading the ceiling at each point.
2. **Forward pass** from rest: at each step, go no faster than acceleration could
   have got you there. Enforces *you cannot speed up faster than the robot can*.
3. **Backward pass** from rest at the end: go no faster than you could still
   brake from to meet the next step. Enforces *you must already be braking before
   the slow bit*.
4. Integrate the speeds to get a time per sample.

The backward pass is the whole point. A naive per-segment profile arrives at a
slow segment still going fast, then brakes harder than the robot physically can.

With a constant ceiling this reproduces the trapezoid, which the tests assert
directly against `MotionProfile`.

---

## The follower

Each `update()`:

1. The profile says where the robot should be by now, as a distance along the
   chain. **This runs on a clock, not on measured progress** — so a path takes the
   same time every run, and a shoved robot does not lose its place in the
   profile, it just develops an error.
2. That distance resolves to a point, tangent and desired heading.
3. Feedforward drives along the tangent at the profiled speed, per wheel.
4. PID adds correction, scaled by reactive authority.
5. Kinematics converts to wheel commands; the set is normalised into powers.

### Reactive blending

```
authority = 1 + (reactiveAuthority − 1) × clamp(error / reactiveErrorScale, 0, 1)
```

On-path, authority sits at 1 and the follower behaves like Road Runner: smooth,
time-consistent. Knocked off, authority ramps and it behaves like Pedro:
correcting hard and immediately.

Blending on error magnitude rather than switching at a threshold is deliberate. A
hard switch puts a discontinuity in commanded velocity exactly where the robot is
already struggling, and a robot hovering near the threshold chatters between two
control regimes.

Authority is also scaled by pose confidence, floored at `minConfidenceAuthority`.

---

## Pose fusion

An EKF over `[x, y, heading]`, in `EKFPoseFuser`. Odometry is the prediction step;
everything else is a measurement update weighted by its own variance. See
[Localization](localization.md) for the reasoning.

Two implementation notes worth knowing before editing it:

**Process noise is built in the robot frame and rotated into the world.** Forward
and lateral odometry have genuinely different error characteristics; building
noise in world coordinates smears the good estimate into the bad one.

**The covariance is re-symmetrised after every update.** A covariance is
symmetric by definition, but repeated floating-point updates let the halves drift
apart by a few ulps. Left alone that compounds and can push the covariance
non-positive-definite, at which point the filter produces nonsense. A 5000-step
test asserts it stays positive and finite.

`Matrix3` is a fixed 3×3 with hand-rolled operations, shared by the EKF and the
RLS tuner. Both are permanently 3×3, so a general linear algebra dependency would
buy nothing.

---

## Online tuning

The feedforward model is linear in its parameters, so every control loop is one
row of a least-squares problem:

```
u = kS·sign(v) + kV·v + kA·a  =  θ·x,   θ = [kS, kV, kA],  x = [sign(v), v, a]
```

Recursive least squares with exponential forgetting. `u` must be
**battery-normalised** (`duty × V_battery / 12`), not raw duty — a duty of 0.5 is
a different applied voltage at 13 V than at 11 V, so a model fitted against raw
duty silently re-fits itself as the battery drains.

### Confidence is not the covariance trace

Worth understanding before changing `getConfidence()`. The obvious implementation
— trace of `P` — is wrong twice over:

- The three regressors differ by orders of magnitude (`sign(v)` ≈ 1, `v` ≈ 50),
  so their variances are not comparable and a trace is dominated by whichever
  parameter carries the smallest regressor.
- Under forgetting, `P` settles at a floor proportional to `1 − λ` rather than
  decaying toward zero. A raw trace can legitimately **grow** as data arrives.

`P` is an inverse information matrix, not a covariance; the covariance is the
residual variance times `P`. So confidence reports
`σ² · Σ P[i][i] · E[x_i²]` — the variance of the predicted command, in command
units for every parameter — against a 2% reference. Measured: 0.00 with no data,
0.29 early, 1.00 on clean data, 0.92 under heavy noise.

### Gains never change mid-path

`TunableFeedforward` holds active and pending sets; the follower commits only in
`followPath()`. The rule is enforced structurally rather than by remembering to
obey it.

---

## Stall detection

```
I_expected = (V_applied − V_backEmf) / R,   R = 12 / I_stall
```

**The back-EMF term uses the velocity the follower commanded, not the one the
encoder reports.** This is the entire trick, and getting it backwards makes the
detector useless — which it was, in the first implementation.

Predicting from measured velocity is self-fulfilling: a wedged wheel reads zero
speed, so the model predicts zero back EMF and therefore a large current, which is
exactly the large current the motor is really drawing. Measured and predicted
agree perfectly and nothing ever looks wrong.

Predicting from commanded velocity asks a useful question instead: *if this wheel
were turning as fast as we told it to, what would it cost?* For a robot cruising
at 30 in/s that is 0.74 A predicted against 5.3 A if jammed — a ratio of 7.

`StallDetectorTest.predictingFromMeasuredVelocityWouldMakeTheDetectorBlind`
guards this specifically, because it is the kind of thing that silently reverts.

---

## Testing

302 tests, all headless.

`SimulatedRobot` is a kinematic stand-in: powers produce wheel velocities
instantly and proportionally. There is no motor dynamics model and no battery
sag, both explicitly out of scope. It tests the follower's **geometry and control
structure** — that it commands the right direction and converges — not that any
gain is right for a real robot.

`ManualClock` lets the whole follower be stepped deterministically with no real
time passing, which is why the suite runs in about a second.

`PlannerExportTest` compiles a verbatim copy of the alliance planner's Java
export. The planner has no compile-time dependency on the library, so this test
is the only thing that would notice the export going stale.

---

## Conventions

- Java 17, no records, no sealed types, no switch patterns — the FTC SDK's
  desugaring accepts this feature set on API 24.
- Value types are immutable with public final fields (`Vector2d`, `Pose2d`,
  `PathState`). They are read in hot loops and getters would be noise.
- Builders for anything with more than three configuration knobs, always taking a
  `DistanceUnit` where distances are involved.
- Degenerate inputs return a defined value rather than NaN — a zero-length path
  tangent returns `Vector2d.ZERO`, not a division by zero — because a NaN reaching
  the motors is unrecoverable and silent.
