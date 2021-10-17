package frc.robot.commands;

import static frc.robot.Constants.spindexerShootSpeed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class SpinupCommand extends CommandBase {

  public final double SPINDEXER_EXTRA_SPIN_UP_SPEED = -0.10;

  public final Shooter shooter;
  public final Accelerator accelerator;
  public final Spindexer spindexer;
  public final double spinUpTime = 1.0;
  public final Timer timer = new Timer();

  public SpinupCommand(Shooter shooter, Accelerator accelerator, Spindexer spindexer) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    this.spindexer = spindexer;

    addRequirements(shooter, accelerator, spindexer);
  }

  @Override
  public void initialize() {
    accelerator.coast();
    spindexer.retractGate();
    spindexer.setOpenLoop(spindexerShootSpeed + SPINDEXER_EXTRA_SPIN_UP_SPEED);
    shooter.setVelocity(Constants.shooterShootSpeed);
    timer.reset();
    timer.start();
    spindexer.retractRamp();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(spinUpTime);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
}
