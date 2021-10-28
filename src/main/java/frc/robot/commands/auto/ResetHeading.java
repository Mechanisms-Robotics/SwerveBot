package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ResetHeading extends InstantCommand {

  public ResetHeading(Rotation2d heading, Swerve swerve) {
    swerve.setHeading(heading);
  }
}
