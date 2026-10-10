# Behavior-tree autos

Every `*.xml` file in this directory becomes an auto in the dashboard chooser, named after the file
(`CitrusSynergyBt.xml` → `CitrusSynergyBt`). Files use the
[BehaviorTree.CPP v4](https://www.behaviortree.dev/) XML format, so they open in Groot2 or the BT
editor. The engine is MW-Lib `com.marswars.bt`. The robot-specific nodes are registered in
`frc.robot.autos.bt.AutoNodes`.

## Robot nodes

| Node | Ports | What it does |
|---|---|---|
| `FollowTrajectory` | `trajectory`, `path_state`=CHOREO_PATH, `end_state`=IDLE, `min_time`=0 | Follows a Choreo path until `isAtChoreoSetpoint()`, then requests `end_state` |
| `WaitForChoreoEvent` | `event` | RUNNING until the running path passes the event. Put it in a `Parallel` next to `FollowTrajectory` |
| `SetIntakeState` / `SetShooterState` / `SetSwerveState` | `state` | `setWantedState(state)` |
| `SetShooterTarget` | `target`=HUB (HUB, LEFT_PASS, RIGHT_PASS) | `ShooterSubsystem.setTarget(...)` |
| `IsAtChoreoSetpoint`, `IsShooterReady`, `IsChassisStationary` | | Conditions |
| `HasChoreoTimeElapsed` | `seconds` | Condition |
| `HasChoreoEventPassed` | `event` | Condition |

MW-Lib adds every BT.CPP v4 built-in that doesn't need scripting: `Sequence`, `ReactiveSequence`,
`SequenceWithMemory`, `Fallback`, `ReactiveFallback`, `Parallel`, `ParallelAll`, `IfThenElse`,
`WhileDoElse`, `Inverter`, `ForceSuccess`, `ForceFailure`, `Repeat`, `RetryUntilSuccessful`,
`KeepRunningUntilFailure`, `Timeout`, `Delay`, `RunOnce`, `SubTree`, `AlwaysSuccess`,
`AlwaysFailure`, `Sleep`, `SetBlackboard` and `UnsetBlackboard`. It also adds `Wait` (seconds) and
`TunableWait` (live-tunable seconds under `/Tuning/<key>`).

## Running an event alongside a path

```xml
<Parallel success_count="1" failure_count="1">
  <FollowTrajectory trajectory="SynergyP1"/>
  <Sequence>
    <WaitForChoreoEvent event="Intake Out"/>
    <SetIntakeState state="INTAKE"/>
    <KeepRunningUntilFailure><AlwaysSuccess/></KeepRunningUntilFailure>  <!-- park -->
  </Sequence>
</Parallel>
```

The parked branch never succeeds. Only the path can end the `Parallel`, which then halts the
branch.

## Checking and debugging

- `./gradlew test` instantiates every file here and fails on unknown nodes, bad ports, or
  trajectories missing from `deploy/choreo`. It also writes `build/bt/nodes.xml`, the node palette
  for the editor and Groot2.
- An XML auto that fails to load still shows up in the chooser. It raises an Elastic alert, and the
  error is logged at `BehaviorTree/<name>/Error`.
- Re-selecting the auto or changing alliance re-reads the file, so copying a new XML to the robot
  doesn't need a code deploy.
- In AdvantageScope, the tree state is at `BehaviorTree/<name>/{Status,Structure,Xml,Result}`.
  `Status` has one letter per node in pre-order: I(dle), R(unning), S(uccess), F(ailure), and
  K for skipped.
- Live view: the robot runs a btlive WebSocket server (`ws://<robot>:1670/`) that the BT editor
  connects to.
