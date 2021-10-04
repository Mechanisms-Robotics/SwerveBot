package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Spindexer;

public class TimedSpindexerCommand extends CommandBase {
  private final Spindexer spindexer;
  private final Accelerator accelerator;

  private final double spindexerWaitTime;
  private final Timer spindexerTimer = new Timer();

  private final double spindexerSpeed;

  public TimedSpindexerCommand(
      Spindexer spindexer,
      Accelerator accelerator,
      double spindexerWaitTime,
      double spindexerSpeed) {
    this.spindexer = spindexer;
    this.accelerator = accelerator;

    this.spindexerWaitTime = spindexerWaitTime;
    this.spindexerSpeed = spindexerSpeed;

    addRequirements(spindexer, accelerator);
  }

  @Override
  public void initialize() {
    spindexer.setOpenLoop(spindexerSpeed);
    if (!spindexer.isGateDeployed()) spindexer.deployGate();
    if (spindexer.isRampDeployed()) spindexer.retractRamp();

    accelerator.coast();
    spindexerTimer.start();
  }

  @Override
  public boolean isFinished() {
    return spindexerTimer.hasElapsed(spindexerWaitTime);
  }

  @Override
  public void end(boolean interrupted) {
    spindexerTimer.stop();
    spindexerTimer.reset();

    spindexer.stop();
  }
}
