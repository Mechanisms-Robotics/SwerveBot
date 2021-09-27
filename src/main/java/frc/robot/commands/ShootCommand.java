package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import java.util.function.Supplier;

public class ShootCommand extends CommandBase {

  private static final double SHOOTER_RPM = 4500 / 8; // RPM TODO: Change to around 4785
  private static final double ACCELERATOR_RPM = 2000 / 8; // RPM

  private final Supplier<Boolean> increaseHoodButtonSupplier;
  private final Supplier<Boolean> decreaseHoodButtonSupplier;

  private final Shooter shooter;
  private final Accelerator accelerator;
  private final Spindexer spindexer;

  public ShootCommand(
      Supplier<Boolean> increaseHoodButtonSupplier,
      Supplier<Boolean> decreaseHoodButtonSupplier,
      Shooter shooter,
      Accelerator accelerator,
      Spindexer spindexer) {
    this.increaseHoodButtonSupplier = increaseHoodButtonSupplier;
    this.decreaseHoodButtonSupplier = decreaseHoodButtonSupplier;

    this.shooter = shooter;
    this.accelerator = accelerator;
    this.spindexer = spindexer;
    addRequirements(shooter, accelerator, spindexer);
  }

  @Override
  public void initialize() {
    shooter.setHoodRawPosition(0.0);
    // TODO: Switch to PID once Tuned
    // shooter.setVelocity(SHOOTER_RPM);
    // accelerator.setVelocity(ACCELERATOR_RPM);

    spindexer.retractGate();
    spindexer.setOpenLoop(0.40);
    shooter.setOpenLoop(0.75);
    accelerator.setOpenLoop(0.70);
  }

  @Override
  public void execute() {
    if (increaseHoodButtonSupplier.get()) {
      shooter.increaseHood();
    } else if (decreaseHoodButtonSupplier.get()) {
      shooter.decreaseHood();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    accelerator.stop();
    spindexer.stop();
    spindexer.deployGate();
  }
}
