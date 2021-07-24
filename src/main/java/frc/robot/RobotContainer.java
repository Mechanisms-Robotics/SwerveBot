// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.subsystems.Swerve;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer {

  // Subsystems
  private final Swerve swerve = new Swerve();

  // The driver's controller
  private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {}

  private void configureDefaultCommands() {
    // Drive the robot relative to the field\
    swerve.setDefaultCommand(new DriveTeleopCommand(
        () -> driverController.getX(Hand.kLeft),
        () -> driverController.getY(Hand.kLeft),
        () -> driverController.getX(Hand.kRight),
        swerve));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void runSwerveCalibration() {
    // Run a non-interruptible command that calibrates the swerve drive.
    CommandScheduler.getInstance().schedule(
      false, new RunCommand(() -> swerve.calibrateSwerveModules(),
      swerve));
  }
}
