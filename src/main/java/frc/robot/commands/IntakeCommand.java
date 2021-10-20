package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import java.util.function.Supplier;

public class IntakeCommand extends CommandBase {
  private final Intake intake;
  private final Spindexer spindexer;
  private final Accelerator accelerator;

  private final Supplier<Boolean> unjam;
  private final Supplier<Boolean> toggleIntake;

  private boolean isIntaking = true;
  private boolean prevToggleIntake = false;

  public IntakeCommand(
      Supplier<Boolean> unjam,
      Supplier<Boolean> toggleIntake,
      Intake intake,
      Spindexer spindexer,
      Accelerator accelerator) {
    this.intake = intake;
    this.toggleIntake = toggleIntake;
    this.spindexer = spindexer;
    this.accelerator = accelerator;
    this.unjam = unjam;
    addRequirements(intake, spindexer, accelerator);
  }

  public IntakeCommand(Intake intake, Spindexer spindexer, Accelerator accelerator) {
    this(null, () -> false, intake, spindexer, accelerator);
  }

  @Override
  public void initialize() {
    intake.deploy();

    accelerator.coast();

    spindexer.deployGate();
    spindexer.retractRamp();
    spindexer.setOpenLoop(Constants.spindexerIntakeSpeed);
  }

  @Override
  public void execute() {
    if (toggleIntake.get() && !prevToggleIntake) {
      isIntaking = !isIntaking;
      prevToggleIntake = true;
    } else if (!toggleIntake.get() && prevToggleIntake) {
      prevToggleIntake = false;
    }

    if (isIntaking) {
      intake.setOpenLoop(Constants.intakeSpeed);
    } else {
      intake.stop();
    }

    if (unjam != null) {
      if (unjam.get()) {
        spindexer.setOpenLoop(-0.15);
      } else {
        spindexer.setOpenLoop(Constants.spindexerIntakeSpeed);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    spindexer.stop(); // TODO: Remove
    intake.stop();
    intake.retract();
  }
}
