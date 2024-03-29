package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ContinuousJogHoodCommand extends CommandBase {

  private Hood hood;
  private boolean reverse;

  public ContinuousJogHoodCommand(Hood hood, boolean reverse) {
    this.hood = hood;
    this.reverse = reverse;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    if (reverse) {
      hood.decreaseHood();
    } else {
      hood.increaseHood();
    }
  }
}
