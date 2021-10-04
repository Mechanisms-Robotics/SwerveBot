package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;

public class IntakeCommand extends CommandBase {
  private final Intake intake;
  private final Spindexer spindexer;
  private final Accelerator accelerator;

  public IntakeCommand(Intake intake, Spindexer spindexer, Accelerator accelerator) {
    this.intake = intake;
    this.spindexer = spindexer;
    this.accelerator = accelerator;
    addRequirements(intake, spindexer, accelerator);
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
  public void end(boolean interrupted) {
    spindexer.stop(); // TODO: Remove
    intake.stop();
    intake.retract();
  }
}
