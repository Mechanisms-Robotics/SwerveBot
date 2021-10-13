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

  public IntakeCommand(Supplier<Boolean> unjam, Intake intake, Spindexer spindexer, Accelerator accelerator) {
    this.intake = intake;
    this.spindexer = spindexer;
    this.accelerator = accelerator;
    this.unjam = unjam;
    addRequirements(intake, spindexer, accelerator);
  }

  public IntakeCommand(Intake intake, Spindexer spindexer, Accelerator accelerator){
    this(null, intake, spindexer, accelerator);
  }


  @Override
  public void initialize() {
    intake.deploy();
    intake.setOpenLoop(Constants.intakeSpeed);

    accelerator.coast();

    spindexer.deployGate();
    spindexer.retractRamp();
    spindexer.setOpenLoop(Constants.spindexerIntakeSpeed);
  }

  @Override
  public void execute() {
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
