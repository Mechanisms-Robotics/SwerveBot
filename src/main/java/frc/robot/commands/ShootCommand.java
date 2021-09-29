package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class ShootCommand extends CommandBase {

  private final Shooter shooter;
  private final Accelerator accelerator;
  private final Spindexer spindexer;

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

    accelerator.setOpenLoop(Constants.acceleratorShootSpeed);
    spindexer.deployRamp();
    spindexer.retractGate();
    spindexer.setOpenLoop(Constants.spindexerShootSpeed);
    shooter.setOpenLoop(Constants.shooterShootSpeed);
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
