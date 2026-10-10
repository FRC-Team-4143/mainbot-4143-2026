# Autos

Every auto is a [BehaviorTree.CPP v4](https://www.behaviortree.dev/) XML file. Each `*.xml` file
in this directory is an option in the dashboard Auto Chooser, named after the file. The file's
`main_tree_to_execute` must use the same name.

| Path | What it is |
|---|---|
| `*.xml` | Autos in the chooser |
| `lib/common.xml` | Shared subtrees (`IntakeOnEvent`, `ShootFor`), pulled in with `<include path="lib/common.xml"/>` |
| `unlisted/*.xml` | Autos kept out of the chooser. They're still checked by tests. Move one up a level to list it |

The engine is MW-Lib `com.marswars.bt`. The robot nodes are registered in
`frc.robot.autos.bt.AutoNodes`. The files open in Groot2 or the BT editor.

## Nodes

Use the standard BT.CPP nodes for flow control and timing:

- **Order and branching:** `Sequence`, `Fallback`, `ReactiveSequence`, `Parallel`, `IfThenElse`
  and the rest of the standard control nodes.
- **Waits:** `Sleep msec` waits. `Delay delay_msec` waits, then runs its child.
- **Decorators:** `Timeout`, `Repeat`, `RetryUntilSuccessful`, `Inverter`, `ForceSuccess`,
  `KeepRunningUntilFailure`, `RunOnce`.
- **Blackboard:** `SetBlackboard` and `SubTree`.

Scripting (`Script`, `_skipIf`, and so on) isn't supported.

The robot nodes:

| Node | Ports | What it does |
|---|---|---|
| `FollowTrajectory` | `trajectory`, `path_state`=CHOREO_PATH, `end_state`=IDLE, `min_time`=0 | Follows a Choreo path until `isAtChoreoSetpoint()` (after at least `min_time` s), then requests `end_state` |
| `WaitForChoreoEvent` | `event` | RUNNING until the running path passes the event |
| `SetIntakeState` / `SetShooterState` / `SetSwerveState` | `state` | `setWantedState(state)` |
| `SetShooterTarget` | `target`=HUB (HUB, LEFT_PASS, RIGHT_PASS) | `ShooterSubsystem.setTarget(...)` |
| `IsAtChoreoSetpoint`, `IsShooterReady`, `IsChassisStationary` | | Conditions |
| `HasChoreoTimeElapsed` | `seconds` | Condition |
| `HasChoreoEventPassed` | `event` | Condition |

## Tunable values (replaces `DynamicWaitCommand`)

To make a value tunable, declare it as a port of the auto's own tree in `<TreeNodesModel>`, then
read it from the blackboard with `{name}`:

```xml
<Delay delay_msec="{middle_wait_msec}">
  <FollowTrajectory trajectory="SynergyP2"/>
</Delay>
...
<TreeNodesModel>
  <SubTree ID="CitrusSynergy">
    <input_port name="middle_wait_msec" type="int" default="3000">Wait for partners</input_port>
  </SubTree>
</TreeNodesModel>
```

- **On the dashboard.** Each declared port appears as `/Tuning/Autos/<auto>/<port>` as soon as the
  robot boots. Edit it from Elastic or AdvantageScope.
- **When the value is read.** The value at the moment the auto starts goes into the blackboard.
- **What carries over.** Edits are logged and replay correctly, but they reset on reboot, so copy
  settled values back into `default`. If you change `default` in the XML and the entry hasn't been
  tuned, the entry follows the new default.
- **Supported types:** `int`, `double`, `bool`, and `std::string`.

## Choreo events

Run event handlers in a `Parallel` next to the path. Only the path should be able to finish it:

```xml
<Parallel success_count="1">
  <FollowTrajectory trajectory="SynergyP1"/>
  <SubTree ID="IntakeOnEvent" event="Intake Out" state="INTAKE"/>
</Parallel>
```

`IntakeOnEvent` waits for the marker, sets the intake state, then stays RUNNING until the path
ends. For other actions, copy its pattern: `WaitForChoreoEvent`, then the actions, then a parked
`KeepRunningUntilFailure`/`AlwaysSuccess`.

## Checking and debugging

- **Run the tests.** `./gradlew test` instantiates every auto, including `unlisted/`. It fails on
  unknown nodes, bad ports, or trajectories missing from `deploy/choreo`. It also writes
  `build/bt/nodes.xml`, the node palette for the editor and Groot2.
- **A broken auto still shows up.** An auto that fails to load is still in the chooser. It raises
  an Elastic alert and logs `BehaviorTree/<name>/Error`.
- **No code deploy needed for XML edits.** Re-selecting an auto or changing alliance re-reads its
  file.
- **In AdvantageScope,** look at `BehaviorTree/<name>/{Status,Structure,Xml,Result}` and
  `BehaviorTree/Active`. `Status` has one letter per node in tree order: I(dle), R(unning),
  S(uccess), F(ailure), and K for skipped.
- **Live view:** the btlive WebSocket server at `ws://<robot>:1670/`.
