package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class ShootCommand extends CommandBase {

  private final Shooter shooter;
  private final Accelerator accelerator;
  private final Spindexer spindexer;

  private final Timer timer = new Timer();
  private static final double ACCEL_TIME = 0.1;

  public ShootCommand(Shooter shooter, Accelerator accelerator, Spindexer spindexer) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    this.spindexer = spindexer;
    addRequirements(shooter, accelerator, spindexer);
  }

  @Override
  public void initialize() {
    // TODO: Switch to PID once Tuned
    // shooter.setVelocity(SHOOTER_RPM);
    // accelerator.setVelocity(ACCELERATOR_RPM);

    // shooter.setVelocity(Constants.shooterShootSpeed);
    accelerator.setOpenLoop(Constants.acceleratorShootSpeed);
    // spindexer.deployRamp();
    spindexer.retractGate();
    spindexer.setOpenLoop(Constants.spindexerShootSpeed);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(ACCEL_TIME)) {
      spindexer.deployRamp();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    accelerator.stop();
    spindexer.stop();
    spindexer.deployGate();
    spindexer.retractRamp();
    timer.stop();
  }
}
