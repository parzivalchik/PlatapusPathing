# Getting started

## Adding it to your project

PlatapusPathing is two modules. `:core` is pure Java — all the math and control, no
Android, no FTC SDK. `:ftc` is a thin Android library holding the hardware
wrappers.

### Into an existing FtcRobotController project

Copy the `core/` and `ftc/` directories into your project root, then add them to
`settings.gradle`:

```gradle
include ':core'
include ':ftc'
```

and depend on `:ftc` from your TeamCode's `build.gradle`:

```gradle
dependencies {
    implementation project(':ftc')
}
```

`:ftc` re-exports `:core`, so you get `Pose2d`, `PathBuilder` and the rest
without adding it separately.

### Standalone

Clone the repo and run the tests — no Android SDK, no emulator, no robot:

```bash
./gradlew :core:test
```

The Android modules are only included in the build when an SDK is actually
present, so this works on a machine that has never had Android Studio installed.

### FTC SDK version

`:ftc` and `:TeamCode` compile against **FTC SDK 11.2.1** (released 2026-07-31),
pinned in their `build.gradle` files. Events run whatever is current, so check
for a newer release before a competition and bump both files together.

Check it against a primary source, not a fork's README or a summary:

- Releases: <https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases>
- The published coordinate, which is what Gradle actually resolves:
  <https://repo1.maven.org/maven2/org/firstinspires/ftc/RobotCore/maven-metadata.xml>

Note the two use different formats — a GitHub tag of `v11.2.1` is the Maven
version `11.2.1`, and a tag like `v11.2` is `11.2.0`. After bumping, run
`./gradlew :ftc:compileDebugJavaWithJavac --refresh-dependencies` before
trusting it; an unresolvable pin fails at Gradle sync, not at runtime.

They depend on the SDK with `compileOnly`, against the published AARs, rather
than vendoring the robot-controller app. That matters for the toolchain: this
repo builds SDK 11.2.1 fine on **AGP 8.7 / Gradle 8.13**, which is what the
wrapper here ships.

The SDK's *own* repository asks for a newer toolchain than that — merging these
modules into an `FtcRobotController` project means adopting **that** project's
Gradle, AGP and Android Studio requirements, not this one's. If your robot
project already builds, adding `:core` and `:ftc` to it will not change what it
needs.

---

## Your first auto

```java
@Autonomous(name = "My Auto")
public class MyAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. Describe the drivetrain. Every number here is measurable or on a
        //    datasheet -- see docs/tuning.md.
        MecanumDrivetrain drivetrain = MecanumDrivetrain.builder(DistanceUnit.INCH)
                .motors(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight")
                .reverse(true, false, true, false)
                .trackWidth(15.0)          // wheel centre to wheel centre, left-right
                .wheelBase(13.0)           // wheel centre to wheel centre, front-back
                .wheelRadius(1.89)         // 96 mm goBILDA mecanum
                .ticksPerRevolution(537.7) // 312 RPM Yellow Jacket, at the output
                .maxMotorRpm(312)
                .build();
        drivetrain.resetEncoders();

        // 2. Say where the robot is. Coordinates are absolute field positions,
        //    so the starting spot has to be measured and given -- it cannot be
        //    worked out from the encoders.
        //
        //    PinpointOdometryComputer and goBILDA's driver both live in
        //    TeamCode, not in this library -- see docs/localization.md.
        OdometryComputer tracker = new PinpointOdometryComputer(
                hardwareMap, "pinpoint",
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                -84.0, -168.0);          // pod offsets from tracking centre, mm

        OdometryComputerLocalizer localizer = OdometryComputerLocalizer.builder(tracker)
                .startPose(new Pose2d(-60, -36, 0))
                .build();

        // 3. Say how hard to drive and how hard to correct.
        FollowerConstants constants = FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(40).maxAcceleration(50).maxDeceleration(60)
                .kV(0.017).kS(0.05)
                .translationalPID(0.15, 0.0, 0.01)
                .headingPID(1.5, 0.0, 0.05)
                .build();

        PathFollower follower = new PathFollower(drivetrain, localizer, constants);

        // 4. Describe the path, in field coordinates. Starting the chain at the
        //    robot's own position means it does not have to drive onto the path.
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(-60, -36), new Point(-36, -36)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                .build();

        waitForStart();

        // 5. Drive it.
        follower.followPath(chain);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("pose", follower.getPose());
            telemetry.update();
        }
    }
}
```

That is the whole API surface for a basic auto: `followPath`, `update`,
`isBusy`, `getPose`.

---

## The loop

`update()` does exactly one control iteration and returns. It never blocks and
never sleeps. Call it as fast as your loop allows — 50 Hz or better is normal,
and the follower measures its own timestep, so an uneven loop rate does not
corrupt the derivative or integral terms.

There is no scheduler, no background thread, and no `run()` that takes over your
OpMode. How you sequence a path against the rest of your auto is yours to
decide — a `while` loop, a state machine, FTCLib commands, Road Runner actions,
or the [optional command layer](commands.md).

---

## Did it work?

`isBusy()` going false means the follower stopped. It does **not** mean it
arrived — the follower also stops when its settle timeout expires, which is a
miss:

```java
telemetry.addData("result", follower.isAtTarget() ? "arrived" : "timed out short");
```

Worth having on telemetry from the first run:

| Value | Reads |
|---|---|
| `getPose()` | Where the library thinks the robot is |
| `getPositionError()` | Distance from the profile's current setpoint |
| `getCorrectionAuthority()` | 1.0 while tracking; rises when fighting back |
| `getPoseConfidence()` | How much the localizer trusts itself — 0 if a pod dropped |

---

## Next

- **First time on hardware?** [BRINGUP.md](../BRINGUP.md) — do this on blocks.
- **Building real paths:** [Paths](paths.md)
- **Making it accurate:** [Tuning](tuning.md)
