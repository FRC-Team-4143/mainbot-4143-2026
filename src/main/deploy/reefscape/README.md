# reefscape (dashboard web app)

Reefscape (2025) dashboard, served off the RoboRIO by `DashboardBridge.startWebServer()`, paired
with `frc.robot.subsystems.reefscapeobserver.ReefscapeObserver`. Tables `/Reefscape/ToRobot` and
`/Reefscape/ToDashboard`, port `5804`.

Uses every `DashboardChannel` shape: bitfield ints (`l2` / `l3` / `l4` / `algae`), a scalar int
(`l1`), bidirectional bools (`coop` / `rp_focus`), and OUTPUT_ONLY bools (`is_elims` /
`rp_complete`). The robot also overrides `coop` / `rp_focus` on mirror-out the way the real game
does.

See `../deepspace/README.md` for how these observers work and how to add another year's.
