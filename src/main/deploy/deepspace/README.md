# FRC 4143 dashboard observers

Browser dashboards served off the RoboRIO by MW-Lib's `DashboardBridge.startWebServer()`. A human
taps a phone/tablet page during a match; the taps round-trip through the robot over NetworkTables.

Each observer is a plain singleton (`getInstance()` + `periodic()` called straight from
`Robot.robotPeriodic()`, so it runs in every mode). It is **not** an `MwSubsystem` and is not
registered in `RobotContainer` -- there is no enum state to transition between. `DashboardBridge`
moves only raw ints and booleans over NT; the game meaning lives entirely in the observer class
and its web app.

## What ships

| game | package | deploy dir | NT tables | port |
|------|---------|-----------|-----------|------|
| 2019 Deep Space | `frc.robot.subsystems.deepspaceobserver` | `src/main/deploy/deepspace/` | `/DeepSpace/ToRobot`, `/DeepSpace/ToDashboard` | 5802 |
| 2023 Charged Up | `frc.robot.subsystems.chargedupobserver` | `src/main/deploy/chargedup/` | `/ChargedUp/ToRobot`, `/ChargedUp/ToDashboard` | 5803 |
| 2025 Reefscape | `frc.robot.subsystems.reefscapeobserver` | `src/main/deploy/reefscape/` | `/Reefscape/ToRobot`, `/Reefscape/ToDashboard` | 5804 |

Both run at the same time. Open `http://<roborio-ip>:<port>/` (or `http://localhost:<port>/` in
sim). The page background is red until NT4 connects, then it flips to the CONNECTED state.

## Files in a deploy dir

| file | change it? | what it is |
|------|------------|------------|
| `index.html` | yes | markup + `class` / `data-*` hooks |
| `index.css` | yes | plain styling |
| `index.js` | yes | NT4 wiring: subscribe `ToDashboard`, publish `ToRobot`, render |
| `NT4.js` | no | NT4 client, vendored verbatim from FRC 6328 (Mechanical-Advantage) |
| `msgpack.js` | no | msgpack codec `NT4.js` depends on, vendored verbatim |

## Adding another year's observer

1. New flat all-lowercase package `frc.robot.subsystems.<game>observer` with one class
   `<Game>Observer` -- copy `DeepSpaceObserver` or `ChargedUpObserver` as the starting point.
2. New `src/main/deploy/<game>/` with `index.{html,css,js}` and `NT4.js` / `msgpack.js` copied
   verbatim from an existing observer's dir.
3. Give it a **distinct** web-server port, a **distinct** `/<Game>/ToRobot` + `/<Game>/ToDashboard`
   table pair, and a **distinct** deploy subdir -- all four args of `DashboardBridge.Config`. Two
   `WebServer`s in one JVM only coexist on different ports.
4. Declare one `DashboardChannel` per NT topic (`bidirectionalInt` / `bidirectionalBool` for things
   the human sets, `outputInt` / `outputBool` for things the robot computes and only reports).
   Channel-name strings must be byte-identical between the Java class and the web app's `index.js`.
5. Bit `i` of a packed int is element `i` of the `boolean[]` -- use `NumUtil.packBits` /
   `NumUtil.unpackBits` and the matching `^ (1 << bit)` / `& (1 << bit)` in `index.js`.
6. Wire `getInstance().periodic()` into `Robot.robotPeriodic()`.
