package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;

public class SpinupShooterCommand extends CommandBase {

  public final double SPINDEXER_EXTRA_SPIN_UP_SPEED = 0.10;

  public final Shooter shooter;
  public final Accelerator accelerator;
  public final double spinupTime = 1.0;
  public final Timer timer = new Timer();

  public SpinupShooterCommand(Shooter shooter, Accelerator accelerator) {
    this.shooter = shooter;
    this.accelerator = accelerator;

    addRequirements(shooter, accelerator);
  }

  @Override
  public void initialize() {
    accelerator.coast();
    shooter.setOpenLoop(Constants.shooterShootSpeed);
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(spinupTime);
  }
}
