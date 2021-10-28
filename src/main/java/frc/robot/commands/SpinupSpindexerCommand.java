package frc.robot.commands;

import static frc.robot.Constants.spindexerShootSpeed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class SpinupSpindexerCommand extends CommandBase {

  public final Accelerator accelerator;
  public final Spindexer spindexer;
  public final double gateRetractTime = 0.5;
  public final Timer timer = new Timer();

  public SpinupSpindexerCommand(Accelerator accelerator, Spindexer spindexer) {
    this.accelerator = accelerator;
    this.spindexer = spindexer;

    addRequirements(accelerator, spindexer);
  }

  @Override
  public void initialize() {
    accelerator.coast();
    spindexer.retractGate();
    spindexer.setOpenLoop(-0.20);
    timer.reset();
    timer.start();
    spindexer.retractRamp();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(gateRetractTime)) {
      spindexer.setOpenLoop(spindexerShootSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
}
