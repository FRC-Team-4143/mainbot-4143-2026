# Autonomous

Autos are **behavior trees in BehaviorTree.CPP v4 XML**, not Java classes. Each
`src/main/deploy/autos/<Name>.xml` file is one auto in the dashboard chooser, named after the file.
Its `main_tree_to_execute` must use the same name. MW-Lib (`com.marswars.bt`) runs them through
the normal `Auto`/`AutoManager`/`CommandScheduler` flow. Full reference:
`src/main/deploy/autos/README.md`.

- **Don't add `Auto` subclasses or command sequences for autos.** Write XML.
- **Robot actions and conditions** are registered in `frc/robot/autos/bt/AutoNodes.java`. Add new
  nodes there, and never call `getInstance()` in a node constructor.
- **Shared subtrees** live in `deploy/autos/lib/common.xml`.
- **Hidden autos** go in `deploy/autos/unlisted/`. They're tested but not in the chooser.

## Anatomy of an auto

```xml
<root BTCPP_format="4" main_tree_to_execute="CitrusSynergy">
  <include path="lib/common.xml"/>
  <BehaviorTree ID="CitrusSynergy">
    <Sequence>
      <SetShooterTarget target="HUB"/>
      <SetIntakeState state="STORE"/>
      <Sleep msec="1000"/>
      <!-- path + Choreo event handler; only the path can finish the Parallel -->
      <Parallel success_count="1">
        <FollowTrajectory trajectory="SynergyP1" end_state="IDLE"/>
        <SubTree ID="IntakeOnEvent" event="Intake Out" state="INTAKE"/>
      </Parallel>
      <!-- tunable wait, then the next path -->
      <Delay delay_msec="{middle_wait_msec}">
        <FollowTrajectory trajectory="SynergyP2" end_state="IDLE"/>
      </Delay>
      <SubTree ID="ShootFor" duration_msec="2000"/>
    </Sequence>
  </BehaviorTree>
  <TreeNodesModel>
    <SubTree ID="CitrusSynergy">
      <input_port name="middle_wait_msec" type="int" default="3000">Wait for partners</input_port>
    </SubTree>
  </TreeNodesModel>
</root>
```

### Key pieces

- **Driving.** `FollowTrajectory trajectory="<ChoreoTraj name>"` selects the path and requests
  `path_state` (CHOREO_PATH, or CHOREO_PATH_ROTATION_LOCK). It finishes when
  `isAtChoreoSetpoint()` holds and at least `min_time` seconds have passed, then requests
  `end_state`. Trajectory names are pre-loaded and alliance-flipped automatically.
- **Choreo events.** Handle them next to the path in a `Parallel success_count="1"` with
  `IntakeOnEvent` or `WaitForChoreoEvent`. There are no global `Trigger` bindings.
- **Waits.** Use BT.CPP `Sleep msec` or `Delay delay_msec`; there are no custom wait nodes.
- **Tunable values.** Declare a port on the auto's tree in `<TreeNodesModel>` and read it as
  `{name}`. It appears on the dashboard at `/Tuning/Autos/<auto>/<port>`. This replaces the old
  `DynamicWaitCommand`.
- **State requests.** `SetIntakeState`, `SetShooterState`, `SetSwerveState`, `SetShooterTarget`.

## Swerve states you'll request

- `FIELD_CENTRIC` — normal teleop driving.
- `FIELD_CENTRIC_ROTATION_LOCK` — field-centric translation, heading locked (e.g. aiming).
- `CHOREO_PATH` — follow the selected trajectory.
- `CHOREO_PATH_ROTATION_LOCK` — follow the path while holding a locked heading.

Alliance flipping is handled by the swerve/localization layer, so author paths for one alliance.

## Checking

`./gradlew test` (`BtAutosParseTest`) instantiates every XML auto, including `unlisted/`. It fails
on unknown nodes, bad ports or trajectories missing from `deploy/choreo`.

> The generated files `ChoreoTraj.java` and `ChoreoVars.java` are excluded from Spotless. Don't
> hand-edit them.
