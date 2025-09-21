As of 9.8.25, REV has created a getControlType() function for REVLib 2026. TrackedController will no longer be recieving updates.

# TrackedController

Wrapper for SparkClosedLoopController that tracks the current control type for logging and debugging.

## Problem

The REV SparkMax API does not provide a way to read back the current control type (velocity, position, voltage, etc.) that was last commanded to the motor controller. This makes it difficult to log what control mode the motor is actually running in, which is critical for debugging and AdvantageKit replay functionality.

## Solution

This class wraps the SparkClosedLoopController and internally tracks the ControlType every time setReference() is called. This allows subsystems to:

- Log the current control type for each motor in AdvantageKit
- Debug control mode issues during development
- Verify that motors are running in the expected control mode
- Replay robot behavior accurately in simulation

## Usage

Replace your SparkClosedLoopController with TrackedController:

```java
// Instead of:
SparkClosedLoopController controller = spark.getClosedLoopController();
controller.setReference(velocity, ControlType.kVelocity);

// Use:
TrackedController controller = new TrackedController(spark.getClosedLoopController());
controller.setTrackedReference(velocity, ControlType.kVelocity);
ControlType currentMode = controller.getControlType(); // Now you can log this!
