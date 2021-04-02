package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class AutoCommand extends CommandBase {
  public abstract void generateTrajectories();

  public abstract void initialize();

  public abstract void execute();

  public abstract void end(boolean interrupted);
}
