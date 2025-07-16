  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot;

  import edu.wpi.first.wpilibj.DriverStation;
  import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.Commands;
  import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
  import frc.robot.elevator.Elevator;

  public class RobotContainer {
    private final CommandXboxController xboxController = new CommandXboxController(0);

    private final Elevator elevatorSubsystem;

    public RobotContainer() {
      // Silience joystick warnings when controller "isn't plugged in/detected" (clears up logs)
      DriverStation.silenceJoystickConnectionWarning(true);
      
      // Create all subsystems
      elevatorSubsystem = new Elevator();

      configureBindings();
    }

    private void configureBindings() {
      // A button: Move to encoder position of 9 motor rotations using MaxMotion profiling
      xboxController.a().onTrue(elevatorSubsystem.goToPosition(9));

      // B button: Run at 500 RPM when held down
      // whileTrue() = start command when pressed, cancel when released
      xboxController.b().whileTrue(elevatorSubsystem.runVelocity(500));

      // X button: Stop - immediately stop the motor
      xboxController.x().onTrue(elevatorSubsystem.stop());
    }

    public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
    }
  }
