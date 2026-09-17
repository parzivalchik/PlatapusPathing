# PlatapusPathing documentation

| Guide | For |
|---|---|
| [Getting started](getting-started.md) | Adding the library to your project and running your first path |
| [Paths](paths.md) | Building path chains: curves, headings, speed caps |
| [Tuning](tuning.md) | Every constant, what it does, and what to set it to |
| [Localization](localization.md) | Odometry, the EKF, and how vision will fold in |
| [Commands](commands.md) | Sequencing an auto — optional, and how to skip it |
| [Holding a pose](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ExampleHoldPositionOpMode.java) | `PoseHolder` — staying put while a mechanism runs |
| [Architecture](architecture.md) | How the layers fit together, for contributors |
| [Migrating](migrating.md) | Coming from Pedro Pathing or Road Runner |
| [Bring-up](../BRINGUP.md) | Getting this onto a real robot for the first time |

API reference: `./gradlew :core:javadoc`, then open
`core/build/docs/javadoc/index.html`.

---

## The two things to know before anything else

**Coordinates are field coordinates.** Origin at field centre, heading CCW from
+X. `new Point(-36, -36)` is a fixed spot, and the library must be told where the
robot starts via the localizer's `startPose`. See
[Paths](paths.md#coordinates).

**Units are yours; angles are radians.** Every builder takes a `DistanceUnit` and
converts at the boundary. Angles are always radians, with no unit switch — use
`Math.toRadians()` at the call site.
