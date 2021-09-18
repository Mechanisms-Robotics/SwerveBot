package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {

  private static final double SHOOTER_RPM = 4500; // RPM
  private static final double ACCELERATOR_RPM = 2000; // RPM

  private final Shooter shooter;
  private final Accelerator accelerator;

  public ShootCommand(Shooter shooter, Accelerator accelerator) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    addRequirements(shooter, accelerator);
  }

  @Override
  public void initialize() {
    shooter.setHoodRawPosition(0.0);
    shooter.setVelocity(SHOOTER_RPM);
    accelerator.setVelocity(ACCELERATOR_RPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    accelerator.stop();
  }
}
