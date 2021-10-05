package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Spindexer;

public class PrepShootCommand extends CommandBase {
  private final Spindexer spindexer;
  private final Accelerator accelerator;

  private static final double PREP_TIME = 2.0;
  private final Timer prepTimer = new Timer();

  public PrepShootCommand(Spindexer spindexer, Accelerator accelerator) {
    this.spindexer = spindexer;
    this.accelerator = accelerator;
    addRequirements(spindexer, accelerator);
  }

  @Override
  public void initialize() {
    accelerator.coast();

    spindexer.retractGate();
    spindexer.deployRamp();
    spindexer.setOpenLoop(Constants.spindexerPrepSpeed);
  }

  @Override
  public boolean isFinished() {
    return prepTimer.hasElapsed(PREP_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    spindexer.stop();
    spindexer.retractRamp();
  }
}
