# Localization

Three implementations behind one interface. The follower cannot tell them apart,
so swapping is a one-line change.

**The Control Hub's built-in IMU is not used anywhere in this library**, and no
adapter for it ships. It is unreliable on newer hubs, and heading error is the
single fastest way to turn a small translation error into a large one — so
rather than offer it with a warning, it is simply absent. Bring your own heading
sensor, or better, an odometry computer that carries its own.

---

## Localizer

```java
public interface Localizer {
    void update();                  // read sensors, advance the estimate
    Pose2d getPose();               // best estimate, field coordinates
    void setPose(Pose2d pose);      // override -- prefer startPose at init
    ChassisSpeeds getVelocity();    // robot-relative
    Twist2d getLastTwist();         // movement over the last update
    double getConfidence();         // [0, 1], 1 = tight estimate
}
```

`getConfidence()` defaults to 1.0. That is the honest answer for dead reckoning:
it has no idea how wrong it is. Only `FusedLocalizer` returns anything else.

**Both need a `startPose`.** Poses are absolute field coordinates, and absolute
coordinates cannot be inferred from encoders. The default is the field centre
facing +X, which is almost certainly not where your robot is — get it wrong and
every path is offset by the same amount.

---

## OdometryComputerLocalizer — recommended

For a goBILDA Pinpoint, a SparkFun OTOS, or anything else that tracks its own
pose. The device reads its odometry pods, fuses them with its **own** onboard
gyro — a separate part from the Control Hub's — and hands back a finished pose.

```java
OdometryComputer tracker = new PinpointOdometryComputer(
        hardwareMap, "pinpoint",
        GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
        -84.0, -168.0);          // pod offsets from the tracking centre, mm

OdometryComputerLocalizer localizer = OdometryComputerLocalizer.builder(tracker)
        .startPose(new Pose2d(-60, -36, 0))
        .build();
```

`OdometryComputer` is a five-method interface in `:core` with no FTC types, so it
fakes cleanly in a unit test. The Pinpoint adapter and goBILDA's driver both live
in `TeamCode/` rather than `:ftc`, because the driver is not on Maven Central —
goBILDA ship it as source. It is vendored there (MIT) so the sample compiles;
`:core` and `:ftc` have no dependency on it.

### Nothing is filtered on top

The device has already fused pods and gyro. Running that through `FusedLocalizer`
as well would add lag to an estimate better than anything the extra filter knows
about — and worse, it would shrink the reported covariance as though two
independent measurements had agreed, when there is only one.

### It fails in ways drive encoders cannot

A pod cable pulls out, the gyro runs away, an I2C read drops. The pose then goes
stale or wrong while staying **numerically plausible**, which is the dangerous
case — nothing looks broken.

So the localizer holds the last pose it believed rather than accepting a reading
the device is disowning, drops `getConfidence()` to zero so the follower scales
its correction authority back instead of driving hard at a position the robot is
not in, and treats a NaN as a fault even when the device claims to be healthy.

`hasFaulted()` latches, because a pod that reconnects leaves the pose offset by
however far the robot moved while it was out and the device reports `READY` again
regardless. `getHealthDetail()` gives the vendor's own status — 
`FAULT_X_POD_NOT_DETECTED` tells you which cable to check. Put it on telemetry.

Whether a fault should abort the auto is the OpMode's call, not the library's.

---

## DriveEncoderLocalizer

Odometry from the drive motors' own encoders, with the IMU optionally overriding
wheel-derived heading.

```java
DriveEncoderLocalizer localizer = DriveEncoderLocalizer.builder(drivetrain, Clock.system())
        .startPose(new Pose2d(-60, -36, 0))
        .headingSource(myHeadingSensor)      // see HeadingSource below
        .build();
```

No extra hardware, and the least accurate option: mecanum wheels slip laterally
and the encoders cannot see it, so drift accumulates over a long auto. Supplying
a `HeadingSource` fixes the worst of it, because heading error is what turns
small translation errors into large ones.

Start here during bring-up. It has fewer moving parts, so when the pose is wrong
you know it is the sensors and not the filter.

---

## FusedLocalizer

An extended Kalman filter over `[x, y, heading]`. Odometry is the prediction
step; the IMU — and later AprilTags — enter as **measurement updates** weighted
by their own variance.

```java
FusedLocalizer localizer = FusedLocalizer.builder(drivetrain, Clock.system())
        .startPose(new Pose2d(-60, -36, 0))
        .headingSource(myHeadingSensor)
        .headingVariance(1e-4)     // ~0.6 degrees of sigma
        .build();
```

Use this when you have drive encoders and a *separate* heading sensor. With an
odometry computer, use `OdometryComputerLocalizer` instead — it is both simpler
and more accurate.

### Why a filter instead of resetPose()

The common FTC approach is to call `resetPose()` when a tag comes into view. That
discards a good odometry estimate in favour of a single frame's observation, and
teleports the robot mid-path when that observation is marginal.

Here a measurement moves the estimate in proportion to how much it deserves to.
Measured over 60 repeated fixes from the same starting error:

| Observation variance | Gap closed |
|---|---|
| 0.25 in² (vague) | 67% |
| 0.05 in² | 84% |
| 0.01 in² (tight) | 96% |

A `resetPose()` override closes 100% every time, regardless of whether the
observation earned it.

### Process noise knows what odometry is bad at

Noise is built in the robot's own frame — where "forward" and "sideways" have
genuinely different error characteristics — then rotated into the world.
`lateralNoisePerInch` defaults five times higher than the forward figure, because
mecanum scrubs sideways and the encoders cannot see it.

Building noise directly in world coordinates would smear the good along-track
estimate into the bad lateral one and lose exactly the information that makes
filtering worth doing.

A useful consequence: because the covariance carries correlations, a **heading**
measurement corrects **position** too. A robot that drove a curve has correlated
position and heading error, so learning the heading says something about where it
is. A `resetPose()`-style override throws that away.

| Setting | Default | Meaning |
|---|---|---|
| `translationalNoisePerInch` | 0.002 | Variance added per inch of forward travel, in² |
| `lateralNoisePerInch` | 0.010 | Same, sideways. Deliberately larger |
| `headingNoisePerRadian` | 0.020 | Variance per radian of rotation, rad² |
| `baseNoisePerSecond` | 1e-6 | Added per second regardless of motion |
| `confidenceScale` | 2.0 | Position sigma, inches, at which confidence reads ~0.5 |

`baseNoisePerSecond` keeps the covariance from collapsing to zero while the robot
sits still, which would make the filter ignore every subsequent measurement.

### Confidence

`getConfidence()` is a bounded summary of positional uncertainty, in [0, 1]. It
feeds the follower's correction authority: reacting aggressively toward a pose
you do not believe is how a robot ends up chasing its own estimation error across
the field. Floored by `minConfidenceAuthority` so a doubted estimate still
corrects somewhat.

---

## HeadingSource

```java
@FunctionalInterface
public interface HeadingSource {
    double getHeadingRadians();
}
```

**No implementation ships.** The Control Hub IMU adapter was removed
deliberately; supply your own from whatever sensor you trust — an odometry
computer's heading, a separately-wired IMU, or two parallel dead wheels via
`(leftDistance − rightDistance) / trackWidth`.

Two things your implementation must get right:

- **Return a continuous angle, not a wrapped one.** If your sensor reports yaw in
  (−180°, 180°], accumulate the differences yourself. Handing the wrapped value
  straight over makes a robot spinning past 180° appear to snap a full turn
  backwards in a single loop.
- **Any fixed offset is fine.** Whatever it reads at the first `update()` is
  calibrated against the configured `startPose` heading. That is what puts the estimate in field
coordinates rather than relative to power-on — the IMU's own zero is arbitrary
and never appears in a pose.

---

## Vision

`VisionPoseSource` is a **stub**. The interface and the `FusedLocalizer` plumbing
that consumes it are finished and tested against scripted observations. The
AprilTag half — camera calibration, tag-field layout, turning a detection into a
field-coordinate pose — is not built.

```java
public interface VisionPoseSource {
    final class Observation {
        public final Pose2d pose;        // field coordinates
        public final double[] variance;  // {x, y, heading}
        public final double timestamp;
    }
    Observation getObservation();        // null when nothing is fresh
}
```

It exists now because the shape of this interface constrains the filter's design,
and getting it wrong would mean rewriting the fusion layer later. In particular,
**an observation must carry its own uncertainty**: a tag seen head-on at two feet
and the same tag glimpsed at the frame's edge at ten feet are not the same
measurement, and a fusion layer that cannot tell them apart has no way to behave
sensibly.

If you implement it:

- Return `null` when there is no fresh detection. The localizer skips the update;
  it never blocks.
- Report poses in **field coordinates**. In a season with wall-mounted tags this
  is the easy direction: a tag's field position is fixed and published, so a
  detection converts straight into an absolute pose with no knowledge of where
  the robot started.

  **BIOBUZZ (2026–27) is not such a season.** Its AprilTag clusters are on the
  underside of the HIVE CELLS, which pivot (Competition Manual §9.6, §9.9), and
  the FTC SDK 12.0 release notes say outright: "since BIOBUZZ AprilTags move,
  they are not suitable for absolute Field Localization." `FusedLocalizer.correctPose()` assumes an observation is a
  fixed, known field pose; a moving tag needs the tag's own current pose
  tracked before a detection can become one. Nothing here is built against it
  yet — this is a caution for whoever starts, not a defect.
- Scale variances with observed range and viewing angle. Constant variances
  defeat the purpose of fusing.

Stale observations are rejected by timestamp. A camera pipeline runs slower than
the control loop and re-serves the same detection for several loops; applying it
repeatedly would let one measurement shrink the covariance over and over and
convince the filter it is far more certain than it has any right to be.
