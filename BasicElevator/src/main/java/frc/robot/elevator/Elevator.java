// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TrackedController;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  
  private final SparkMax spark; // Main motor controller
  private final RelativeEncoder encoder; // Built-in NEO encoder for position feedback
  private final TrackedController controller; // Wrapper that tracks control types for logging

  // Configuration object - stores all SparkMax settings
  private final SparkMaxConfig config;

  // Control behavior - brake mode holds position when stopped (important for elevators!)
  private final boolean brakeModeEnabled = true;

  public Elevator() {
    // Initialize hardware
    spark = new SparkMax(Constants.elevatorCAN, MotorType.kBrushless);
    encoder = spark.getEncoder();

    // TrackedController wraps the SparkMax controller to enable control type tracking
    // IMPORTANT TO ONLY USE TRACKEDCONTROLLER FOR COMPLETEFUNCTIONALITY, USING SparkClosedLoopController WILL NOT WORK
    controller = new TrackedController(spark.getClosedLoopController());

    // Create configuration object for all SparkMax settings
    config = new SparkMaxConfig();

    // Brake mode prevents elevator from falling when no power is applied
    config
      .idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure closed-loop control (PID + Motion Profiling)
    config
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // Use the built-in NEO encoder
      .p(0.1) // Proportional gain - how hard to correct position errors 
      .i(0) // Integral gain - eliminates steady-state error
      .d(0) // Derivative gain - dampens oscilations
      .maxMotion
      .maxAcceleration(4000) // How fast to accelerate/decelerate (RPM/sec)
      .maxVelocity(4000) // Maximum velocity during motion (RPM)
      .allowedClosedLoopError(1); // How close to target before considering "at setpoint"


    // Apply all configuration to the SparkMax hardware
    // ResetSafeParameters: Only reset critical safety settings
    // PersistParameters: Save settings to flash memory (survives power cycles)  
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*
   * Move elevator to a specific position using smooth MaxMotion profiling.
   * 
   * @param setpoint Target position in motor rotations
   * @return Command that sets the target once and finishes immediately 
   */
  public Command goToPosition(double setpoint) {
    // TrackedController logs that we're using kMaxMotionPositionControl mode
    return Commands.runOnce(() -> controller.setTrackedReference(setpoint, ControlType.kMAXMotionPositionControl)).withName("Go to position" + setpoint);
  }
  
  /*
   * Run elevator at a constant velocity (manual control).
   * 
   * @param velocityRPM Target velocity in RPM
   * @return Command that runs velocity control while active, runs stop command when cancelled 
   */
  public Command runVelocity(double velocityRPM) {
    // TrackedController logs that we're using kVelocity mode
    return Commands.runEnd(() -> controller.setTrackedReference(velocityRPM, ControlType.kVelocity), () -> stop()).withName("Run velocity" + velocityRPM);
  }

  /*
   * Stop the elevator motor.
   * 
   * @return Command that stops the motor once
   */
  public Command stop() {
    // elevator should hold position due to brake mode
    return Commands.runOnce(() -> spark.stopMotor()).withName("Stop Elevator");
  } 

  @Override
  public void periodic() {
    // This method runs every 20ms while the robot is enabled 

    // Log TrackedController benefits - this data is impossible to get from SparkClosedLoopController!
    // Shows whether any command has been sent yet
    SmartDashboard.putBoolean("Elevator/HasControlType?", controller.hasControlType());

    // Shows what control mode is currently active (kMaxMotionPositionControl, kVelocity, etc)
    // Without TrackedController, this couldn't exist
    SmartDashboard.putString("Elevator/ControlType", controller.getControlType() != null ? controller.getControlType().toString() : "None");
  }
}
