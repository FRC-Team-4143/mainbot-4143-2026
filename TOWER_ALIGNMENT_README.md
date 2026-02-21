# Tower Alignment System - Time-of-Flight Sensors

## Overview

The climber subsystem now includes two Time-of-Flight (ToF) sensors that enable precise alignment with the tower's vertical support structure during climb approach. This system uses a "line following" approach to automatically correct the robot's lateral position while approaching the tower.

## Hardware Configuration

### Sensor Placement
- **Left Sensor**: Mounted on the left side of the climber mechanism
- **Right Sensor**: Mounted on the right side of the climber mechanism
- Both sensors should be positioned to detect the vertical tower support when approaching

### CAN IDs
Configure the following in `ClimberConstants.java`:
```java
public final int LEFT_SENSOR_ID = 20;   // Update with actual CAN ID
public final int RIGHT_SENSOR_ID = 21;  // Update with actual CAN ID
```

## How It Works

### Principle
The two ToF sensors measure the distance to the tower's vertical support. When the robot is centered:
- Both sensors read approximately the same distance
- The alignment error is near zero

When the robot is off-center:
- One sensor reads a larger distance than the other
- The system calculates a correction value to strafe the robot back into alignment

### Alignment Error Calculation
```
alignment_error = left_sensor_range - right_sensor_range

Positive error → Robot is too far LEFT → Strafe RIGHT
Negative error → Robot is too far RIGHT → Strafe LEFT
```

## Configuration Parameters

### In `ClimberConstants.java`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SENSOR_MODE` | `RangeMode.SHORT` | Short range mode for close-range precision |
| `SENSOR_ALIGNMENT_DEADBAND` | 5.0 mm | How close sensors must be to consider aligned |
| `SENSOR_MAX_VALID_RANGE` | 300.0 mm | Maximum distance to consider tower detected |
| `SENSOR_ALIGNMENT_KP` | 0.01 | **Proportional gain for alignment correction (TUNE THIS!)** |
| `SENSOR_RANGE_LIMIT` | 50.0 mm | Minimum safe distance to tower |

### Tuning the Alignment

The most important parameter to tune is `SENSOR_ALIGNMENT_KP`:
- **Too low**: Slow, sluggish corrections
- **Too high**: Oscillation, unstable behavior
- **Just right**: Smooth, responsive alignment

Start with 0.01 and adjust in increments of 0.005 based on testing.

## Usage

### Method 1: Using the AlignToTowerCommand

The `AlignToTowerCommand` provides automated alignment while approaching the tower:

```java
// Simple usage with defaults (0.3 m/s forward, 0.5 m/s max strafe)
Command alignCommand = new AlignToTowerCommand();

// Custom speeds and behavior
Command customAlignCommand = new AlignToTowerCommand(
    0.4,      // Forward speed (m/s)
    0.6,      // Max strafe correction speed (m/s)
    true,     // Stop when aligned
    true      // Maintain current heading
);

// Continuous alignment (doesn't stop when aligned)
Command continuousAlign = new AlignToTowerCommand(
    0.3,      // Forward speed
    0.5,      // Max strafe speed
    false,    // Don't stop when aligned - run until interrupted
    true      // Maintain heading
);
```

### Method 2: Manual Integration

For custom control, use the ClimberSubsystem methods directly:

```java
ClimberSubsystem climber = ClimberSubsystem.getInstance();

// In your periodic/execute method:
climber.updateSensorReadings();  // Updates sensor values

// Check alignment status
if (climber.isAlignedWithTower()) {
    // Robot is aligned - proceed with climb
}

// Get alignment status string
String status = climber.getAlignmentStatus();
// Returns: "ALIGNED", "MOVE_RIGHT", "MOVE_LEFT", or "NO_TOWER_DETECTED"

// Get correction value for manual control
double maxStrafe = 0.5;  // m/s
double correction = climber.getLateralCorrectionValue(maxStrafe);
// Use this correction value with your drivetrain

// Check if sensors are detecting the tower
if (climber.areSensorsDetectingTower()) {
    // Both sensors have valid readings
}

// Get individual sensor readings
double leftDistance = climber.getLeftSensorRange();   // in mm
double rightDistance = climber.getRightSensorRange(); // in mm
double error = climber.getAlignmentError();           // in mm
```

### Method 3: Integrating with Auto/Commands

Example of creating a climb sequence:

```java
Command climbSequence = Commands.sequence(
    // 1. Deploy climber
    Commands.runOnce(() -> climber.setWantedState(ClimberStates.DEPLOY)),
    Commands.waitUntil(climber::isDeployed),
    
    // 2. Use vision to get close to tower
    new DriveToTowerWithVision(),
    
    // 3. Use ToF sensors for precision alignment
    new AlignToTowerCommand(0.2, 0.4, true, true),
    
    // 4. Final approach at slow speed
    new AlignToTowerCommand(0.1, 0.3, false, true)
        .withTimeout(2.0),  // Stop after 2 seconds
    
    // 5. Execute climb
    Commands.runOnce(() -> climber.setWantedState(ClimberStates.L1_CLIMB)),
    Commands.waitSeconds(3.0)
);
```

## Sensor Data Logging

The sensors are automatically logged through the subsystem's periodic update. Check your logs for:
- `Climber/LeftSensor/Range` - Left ToF sensor reading (mm)
- `Climber/RightSensor/Range` - Right ToF sensor reading (mm)
- `Climber/AlignmentError` - Calculated alignment error (mm)
- `Climber/AlignmentStatus` - Current alignment status string

## Troubleshooting

### Sensors Not Detecting Tower
- Check `SENSOR_MAX_VALID_RANGE` - may need to increase
- Verify sensor mounting and field of view
- Check for obstructions in sensor path

### Robot Oscillates During Alignment
- Reduce `SENSOR_ALIGNMENT_KP` value
- Check `SENSOR_ALIGNMENT_DEADBAND` - may be too small

### Alignment Too Slow
- Increase `SENSOR_ALIGNMENT_KP` value
- Increase `maxStrafeSpeed` parameter in command

### One Sensor Always Reads Max Range
- Check sensor wiring and CAN ID
- Verify sensor is mounted correctly
- Check for physical damage to sensor

### Robot Drifts When Aligned
- Check that both sensors are level and parallel
- Verify `SENSOR_ALIGNMENT_DEADBAND` is appropriate
- Consider mechanical factors (bumpy field, wheel slip)

## Safety Considerations

1. **Always test at low speeds first** - Start with 0.1-0.2 m/s forward speed
2. **Monitor sensor readings** - Use dashboard to verify sensors are working
3. **Have manual override ready** - Driver should be able to take control
4. **Set appropriate timeouts** - Don't let alignment run indefinitely
5. **Test minimum safe distance** - Verify `SENSOR_RANGE_LIMIT` prevents collisions

## Advanced Usage

### Custom Alignment Logic

You can create custom alignment behaviors by accessing the sensor data:

```java
public class CustomTowerAlign extends Command {
    private ClimberSubsystem climber = ClimberSubsystem.getInstance();
    
    @Override
    public void execute() {
        climber.updateSensorReadings();
        
        double leftRange = climber.getLeftSensorRange();
        double rightRange = climber.getRightSensorRange();
        double error = climber.getAlignmentError();
        
        // Custom logic here
        if (Math.abs(error) > 10.0) {
            // Large error - aggressive correction
        } else if (Math.abs(error) > 2.0) {
            // Small error - gentle correction
        } else {
            // Aligned - no correction needed
        }
    }
}
```

### Combining with Other Sensors

The ToF alignment can work alongside:
- **Vision**: Use vision for initial approach, ToF for fine alignment
- **Gyro**: Maintain heading while aligning laterally
- **Encoders**: Track distance traveled during alignment

## Implementation Notes

- Sensor readings are updated every control loop in `updateLogic()`
- The sensors are included in the subsystem's IO list for automatic logging
- Deprecated methods (`setSensors()`, `checkSensors()`) maintained for backwards compatibility
- The system assumes symmetrical sensor mounting for accurate alignment
