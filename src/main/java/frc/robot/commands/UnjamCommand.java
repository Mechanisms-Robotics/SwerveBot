package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Spindexer;

public class UnjamCommand extends CommandBase {
  private final Spindexer spindexer;

  public UnjamCommand(Spindexer spindexer) {
    this.spindexer = spindexer;

    addRequirements(spindexer);
  }

  @Override
  public void execute() {
    spindexer.setOpenLoop(-Constants.spindexerIntakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    spindexer.stop();
  }
}
