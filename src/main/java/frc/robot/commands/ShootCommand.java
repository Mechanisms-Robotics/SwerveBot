package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class ShootCommand extends CommandBase {

  private static final double SHOOTER_RPM = 4500 / 8; // RPM
  private static final double ACCELERATOR_RPM = 2000 / 8; // RPM

  private final Shooter shooter;
  private final Accelerator accelerator;
  //private final Spindexer spindexer;

  public ShootCommand(Shooter shooter, Accelerator accelerator) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    //this.spindexer = spindexer;
    addRequirements(shooter, accelerator);
  }

  @Override
  public void initialize() {
    shooter.setHoodRawPosition(0.0);
    //spindexer.setOpenLoop(0.40);

    // TODO: Switch to PID once Tuned
    // shooter.setVelocity(SHOOTER_RPM);
    // accelerator.setVelocity(ACCELERATOR_RPM);
    shooter.setOpenLoop(0.50);
    accelerator.setOpenLoop(0.70);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    accelerator.stop();
    //spindexer.stop();
  }
}
