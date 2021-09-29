package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class FastShootCommand extends CommandBase {

  private final Shooter shooter;
  private final Spindexer spindexer;
  private final Accelerator accelerator;

  private final double spindexerPrepTime = 2.0;
  private final Timer spindexerPrepTimer = new Timer();

  private final double acceleratorSpinupTime = 1.0;
  private final Timer acceleratorSpinupTimer = new Timer();

  private boolean spindexerPrepped = false;

  public FastShootCommand(Shooter shooter, Accelerator accelerator, Spindexer spindexer) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    this.spindexer = spindexer;
    addRequirements(shooter, accelerator, spindexer);
  }

  @Override
  public void initialize() {
    // TODO: Switch to PID once Tuned
    spindexer.deployRamp();
    spindexer.retractGate();
    spindexer.setOpenLoop(Constants.spindexerShootSpeed);
    shooter.setOpenLoop(Constants.shooterShootSpeed);
    spindexerPrepTimer.start();
  }

  @Override
  public void execute() {
    if (spindexerPrepTimer.hasElapsed(spindexerPrepTime)) {
      spindexerPrepTimer.stop();
      spindexerPrepTimer.reset();

      spindexer.stop();
      spindexer.retractRamp();
      accelerator.setOpenLoop(Constants.acceleratorShootSpeed);
      acceleratorSpinupTimer.start();
    } else if (acceleratorSpinupTimer.hasElapsed(acceleratorSpinupTime)) {
      acceleratorSpinupTimer.stop();
      acceleratorSpinupTimer.reset();

      spindexer.deployRamp();
      spindexer.setOpenLoop(Constants.spindexerShootSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    accelerator.stop();
    spindexer.stop();
    spindexer.deployGate();
    spindexer.retractRamp();
  }
}
