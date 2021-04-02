// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoChooser;
import frc.robot.subsystems.Swerve;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer {

  private final Swerve swerve = new Swerve();

  private final AutoChooser autoChooser = new AutoChooser(swerve);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getAutoChooser().getSelected();
  }
}
