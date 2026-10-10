# Autos

Every auto is a behavior tree, written as a [BehaviorTree.CPP v4](https://www.behaviortree.dev/)
XML file. Each `*.xml` file in this folder is an option in the dashboard Auto Chooser, named
after the file. The trees run on MW-Lib's behavior-tree engine (`com.marswars.bt`), and the files
open in Groot2 or the BT editor.

| Path | What it is |
|---|---|
| `*.xml` | Autos in the chooser |
| `unlisted/*.xml` | Finished autos kept out of the chooser. They're still checked by the tests. Move one up a level to list it |
| `lib/common.xml` | Shared subtrees every auto can use (see [Shared subtrees](#shared-subtrees)) |
| `lib/nodes.xml` | Generated list of every node an auto can use, for the editor and Groot2 (see [Node palette](#node-palette)) |

## Anatomy of an auto

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="CitrusSynergy">
  <include path="lib/common.xml"/>

  <BehaviorTree ID="CitrusSynergy">
    <Sequence>
      <SetShooterTarget target="HUB"/>
      <SetIntakeState state="STORE"/>
      <Sleep msec="1000"/>

      <!-- drive the first path; turn the intake on when it passes "Intake Out" -->
      <ParallelDeadline>
        <FollowTrajectory trajectory="SynergyP1" end_state="IDLE"/>
        <SubTree ID="IntakeOnEvent" event="Intake Out" state="INTAKE"/>
      </ParallelDeadline>

      <!-- wait a tunable time, then drive the second path -->
      <Delay delay_msec="{middle_wait_msec}">
        <FollowTrajectory trajectory="SynergyP2" end_state="IDLE"/>
      </Delay>

      <SubTree ID="ShootFor" duration_msec="2000"/>
    </Sequence>
  </BehaviorTree>

  <!-- tunable values for this auto (see "Tunable values") -->
  <TreeNodesModel>
    <SubTree ID="CitrusSynergy">
      <input_port name="middle_wait_msec" type="int" default="3000">Wait for partners</input_port>
    </SubTree>
  </TreeNodesModel>
</root>
```

Rules for every file:

- **Names match.** The file name, `main_tree_to_execute`, and the `<BehaviorTree ID>` are all the
  auto's name.
- **Shared blocks come from `lib/common.xml`.** Pull them in with `<include path="lib/common.xml"/>`,
  or `../lib/common.xml` from `unlisted/`, and call them with `<SubTree ID="...">`. A block only
  one auto needs can be another `<BehaviorTree>` in that auto's own file.
- **Attributes are ports.** A literal (`msec="1000"`) is used as written. `{name}` reads the
  blackboard entry `name`.

## Adding an auto

1. Make the paths in Choreo. They end up in `deploy/choreo/<name>.traj`.
2. Copy an existing auto to `deploy/autos/<NewName>.xml`, and rename the file, the
   `main_tree_to_execute` and the `<BehaviorTree ID>`.
3. Edit the steps. Every path named in a `trajectory="..."` is loaded and flipped for the
   alliance automatically.
4. Run `./gradlew test`. It loads every auto and fails on typos, unknown nodes, bad port values,
   or paths missing from `deploy/choreo`.

## Nodes

### Flow control and timing (BehaviorTree.CPP built-ins)

| Node | What it does |
|---|---|
| `Sequence` | Run children in order. Stops at the first failure |
| `Fallback` | Try children in order until one succeeds |
| `ReactiveSequence` / `ReactiveFallback` | Same, but re-check earlier children every tick |
| `Parallel success_count failure_count` | Run all children together; finish on those counts |
| `ParallelAll max_failures` | Run all children to completion |
| `IfThenElse` / `WhileDoElse` | Branch on the first child |
| `Sleep msec` | Wait |
| `Delay delay_msec` | Wait, then run its child |
| `Timeout msec` | Stop the child and fail if it takes too long |
| `Repeat num_cycles`, `RetryUntilSuccessful num_attempts` | Loop the child |
| `Inverter`, `ForceSuccess`, `ForceFailure`, `KeepRunningUntilFailure`, `RunOnce` | Change the child's result |
| `SetBlackboard`, `UnsetBlackboard` | Write or clear blackboard entries |
| `SubTree ID` | Run another `<BehaviorTree>`, such as one from `lib/common.xml`. Its other attributes are its ports |

BT.CPP scripting (`Script`, `_skipIf`, `Precondition`, and so on) isn't supported.

### MW-Lib shared nodes (any robot)

| Node | Ports | What it does |
|---|---|---|
| `ParallelDeadline` | | Runs its children together until the **first** child finishes, then stops the rest and returns that child's result |
| `FollowTrajectory` | `trajectory`, `path_state`=CHOREO_PATH, `end_state`=IDLE, `min_time`=0 | Drives a Choreo path until `isAtChoreoSetpoint()` (and at least `min_time` s), then requests `end_state` |
| `WaitForChoreoEvent` | `event` | Waits until the running path passes a Choreo event marker |
| `SetSwerveState` | `state` | Requests a swerve state |
| `IsAtChoreoSetpoint`, `IsChassisStationary` | | Conditions |
| `HasChoreoTimeElapsed` | `seconds` | Condition |
| `HasChoreoEventPassed` | `event` | Condition |

### Robot nodes (`frc.robot.autos.bt.AutoNodes`)

| Node | Ports | What it does |
|---|---|---|
| `SetWantedState` | `state` (a `RobotStates`) | `RobotContainer.setWantedState(state)`, then RUNNING until the robot is in the state (SUCCESS) or the state fails (FAILURE). Use it for anything teleop also does, so the sequence is written once |
| `IsSystemState` | `state` | The robot (RobotContainer) is in `state` |
| `SetIntakeState` / `SetShooterState` | `state` | `setWantedState(state)` on that subsystem, for small one-off moves |
| `SetShooterTarget` | `target`=HUB (HUB, LEFT_PASS, RIGHT_PASS) | `ShooterSubsystem.setTarget(...)` |
| `IsShooterReady` | | Condition |

Nodes any robot could use belong in MW-Lib. Only robot-specific ones go in `AutoNodes`.

## Paths and Choreo events

Put the path first in a `ParallelDeadline` and the event handlers after it. The path decides when
the group ends, and any handler still waiting is stopped:

```xml
<ParallelDeadline>
  <FollowTrajectory trajectory="SlopAutoStart" end_state="FIELD_CENTRIC_ROTATION_LOCK"/>
  <SubTree ID="IntakeOnEvent" event="Intake Out" state="INTAKE"/>
  <SubTree ID="IntakeOnEvent" event="Intake In" state="STORE"/>
</ParallelDeadline>
```

A path with no events is just a `FollowTrajectory`. `WaitForChoreoEvent`, which `IntakeOnEvent`
uses, only works while its path is running, so keep handlers inside the `ParallelDeadline`. For an
action other than the intake, use a `Sequence` of `WaitForChoreoEvent` followed by the actions.

## Shared subtrees

`lib/common.xml` holds small blocks that several autos use. It isn't an auto itself, so it isn't
in the chooser.

| Subtree | Ports | What it does |
|---|---|---|
| `IntakeOnEvent` | `event`, `state`=INTAKE | Waits for a Choreo event on the running path, then sets the intake state |
| `ShootFor` | `duration_msec`=2000 | Robot `SHOOT` (the driver's right trigger: feeds once the shooter is ready) for `duration_msec`, then robot `AUTO` |

To add one, write a `<BehaviorTree ID="...">` in `lib/common.xml` that reads its inputs as
`{name}`. Declare those inputs in the file's `<TreeNodesModel>` as a `<SubTree ID="...">` with
ports, so editors and Groot2 show them.

## Tunable values

Any value can be tuned from the dashboard without a code deploy. Declare it as a port of the
auto's tree in `<TreeNodesModel>`, then read it with `{name}`:

```xml
<Delay delay_msec="{middle_wait_msec}"> ... </Delay>
...
<TreeNodesModel>
  <SubTree ID="CitrusSynergy">
    <input_port name="middle_wait_msec" type="int" default="3000">Wait for partners</input_port>
  </SubTree>
</TreeNodesModel>
```

- **On the dashboard.** Each value appears at `/Tuning/Autos/<auto>/<name>` as soon as the robot
  boots. Edit it in Elastic or AdvantageScope.
- **When it's read.** The value when the auto starts is the one used for that run.
- **What persists.** Edits are logged and replay correctly, but they reset on reboot. Copy a value
  you want to keep into `default`. If you change `default` and nobody has edited the dashboard
  value, the dashboard follows the new default.
- **Types:** `int`, `double`, `bool`, `std::string`. Times are in milliseconds, like BT.CPP's
  `Sleep` and `Delay`.

## Testing and debugging

- **`./gradlew test`** loads every auto, including `unlisted/` and the `lib/common.xml` subtrees, with the robot's real node set.
  It checks every trajectory file exists, that the chooser lists exactly the expected autos, and
  that `lib/nodes.xml` is up to date.
- **Broken files still show up.** An auto that fails to load still appears in the chooser. It
  raises an Elastic alert and logs `BehaviorTree/<auto>/Error`, and it does nothing when run.
- **No code deploy for XML changes.** Re-selecting an auto, or changing alliance, re-reads its
  file.
- **AdvantageScope.** `BehaviorTree/Active` is the running auto.
  `BehaviorTree/<auto>/{Status,Structure,Xml,Result}` describe the tree and its state. `Status`
  has one letter per node in tree order: I(dle), R(unning), S(uccess), F(ailure), K (skipped).
- **Run logs.** Every run writes `<auto>_<date>_<time>.btlog`, in the same binary format as
  BehaviorTree.CPP's `FileLogger2`. Open it in Groot2 or the BT editor to replay the run node by
  node. It records every status change, including ones too quick to show in `Status`.
  - **Where.** `/U/logs/bt` on the robot, next to the AdvantageKit logs, and `logs/bt` in sim.
    Log replay writes `_replay` copies.
  - **Timing.** Times follow the same clock as the AdvantageKit log.
  - **Crash-safe.** It's written as the auto runs, so a power loss mid-auto still leaves a usable
    file.
- **Live view.** The robot streams the running tree to the BT editor over WebSocket at
  `ws://<robot>:1670/`, using the btlive protocol.

## Node palette

`lib/nodes.xml` lists every node an auto can use, with its ports, defaults and descriptions, for
the editor and Groot2. It starts with MW-Lib's shared nodes, which MW-Lib also ships as
`com/marswars/bt/mwlib_nodes.xml`, followed by this robot's nodes. It's generated from the Java
code. After adding or changing a node, regenerate it:

```
./gradlew test -PupdateNodeSpec
```
