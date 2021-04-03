// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.auto.Slalom;
import frc.robot.subsystems.Swerve;
import java.util.Map;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer {

  private final Swerve swerve = new Swerve();

  private enum AutoCommandSelector {
    NONE,
    SLALOM
  }

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  private AutoCommandSelector selectAutoCommand() {
    return AutoCommandSelector.NONE;
  }

  public Command getAutonomousCommand() {
    return new SelectCommand(
        Map.ofEntries(
            Map.entry(AutoCommandSelector.NONE, null),
            Map.entry(AutoCommandSelector.SLALOM, new Slalom(swerve))),
        this::selectAutoCommand);
  }
}
