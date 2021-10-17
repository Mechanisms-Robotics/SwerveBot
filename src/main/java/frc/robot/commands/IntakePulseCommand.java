package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import java.util.function.Supplier;

public class IntakePulseCommand extends CommandBase {
  private final Intake intake;
  private final Spindexer spindexer;
  private final Accelerator accelerator;

  private final Supplier<Boolean> unjam;

  private final Timer spindexerRunTimer = new Timer();
  private final Timer spindexerStopTimer = new Timer();
  private final double spindexerPulseTime = 0.5; // seconds
  private boolean spindexerIsRunning = true;

  public IntakePulseCommand(
      Supplier<Boolean> unjam, Intake intake, Spindexer spindexer, Accelerator accelerator) {
    this.intake = intake;
    this.spindexer = spindexer;
    this.accelerator = accelerator;
    this.unjam = unjam;
    addRequirements(intake, spindexer, accelerator);
  }

  public IntakePulseCommand(Intake intake, Spindexer spindexer, Accelerator accelerator) {
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

    spindexerRunTimer.start();
  }

  @Override
  public void execute() {
    if (unjam != null) {
      if (unjam.get()) {
        spindexer.setOpenLoop(-0.15);
        spindexer.retractGate();
        System.out.println("RETRACTED GATE");
      } else {
        if (!spindexer.isGateDeployed()) spindexer.deployGate();
        if (spindexerIsRunning) {
          spindexer.setOpenLoop(Constants.spindexerIntakeSpeed);

          if (spindexerRunTimer.hasElapsed(spindexerPulseTime)) {
            spindexerRunTimer.stop();
            spindexerRunTimer.reset();
            spindexerStopTimer.start();

            spindexerIsRunning = false;
          }
        } else {
          spindexer.stop();

          if (spindexerStopTimer.hasElapsed(spindexerPulseTime)) {
            spindexerStopTimer.stop();
            spindexerStopTimer.reset();
            spindexerRunTimer.start();

            spindexerIsRunning = true;
          }
        }
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
