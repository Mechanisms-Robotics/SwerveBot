package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.PS4Controller;
import frc.robot.util.PS4Controller.Direction;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer {

  // Subsystems
  public final Swerve swerve = new Swerve();
  // private final Shooter shooter = new Shooter();
  // private final Hood hood = new Hood();
  private final Accelerator accelerator = new Accelerator();
  private final Spindexer spindexer = new Spindexer();
  // private final Climber climber = new Climber();
  private final Intake intake = new Intake();

  // The driver's controller
  private final PS4Controller driverController = new PS4Controller(0);
  private final PS4Controller operatorController = new PS4Controller(1);

  // Every button for controlling the robot.
  private final Button spinupTrigger = new Button(driverController::getLeftTriggerButton);
  private final Button shootTrigger = new Button(driverController::getRightTriggerButton);
  private final Button prepShootButton = new Button(driverController::getCircleButton);
  private final Button justShootButton = new Button(driverController::getSquareButton);
  private final Button hoodJogForward = new Button(driverController::getRightBumperButton);
  private final Button hoodJogReverse = new Button(driverController::getLeftBumperButton);
  private final Button intakeButton = new Button(driverController::getXButton);

  private final Button climbUpButton = new Button(() -> driverController.getPOV() == Direction.Up);
  private final Button climbDownButton =
      new Button(() -> driverController.getPOV() == Direction.Down);

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    /*
    spinupTrigger.whenPressed(new SpinupCommand(shooter, accelerator, spindexer));
    shootTrigger.whenHeld(new ShootCommand(shooter, accelerator, spindexer));

    prepShootButton.whenPressed(new PrepShootCommand(spindexer, accelerator));
    justShootButton.toggleWhenPressed(
        new SequentialCommandGroup(
            new SpinupShooterCommand(shooter, accelerator),
            new ShootCommand(shooter, accelerator, spindexer)));

    hoodJogForward.whenHeld(new ContinuousJogHoodCommand(hood, false));
    hoodJogReverse.whenHeld(new ContinuousJogHoodCommand(hood, true));

    // TODO: Don't have icky magic number
    */ intakeButton.toggleWhenPressed(
        new IntakeCommand(intake, spindexer, accelerator)
            .andThen(
                new TimedSpindexerCommand(
                    spindexer, accelerator, 5.0, Constants.spindexerIntakeSpeed)));
    /// climbUpButton.whenHeld(
    //    new StartEndCommand(() -> climber.setOpenLoop(0.25), climber::stop, climber));
    // climbDownButton.whenHeld(
    //    new StartEndCommand(() -> climber.setOpenLoop(-0.25), climber::stop, climber));
    // intakeButton.whenPressed(new IntakeCommand(intake, spindexer, accelerator));
    // intakeRetractButton.whenPressed(new InstantCommand(intake::retract, intake));
  }

  private void configureDefaultCommands() {
    // Drive the robot relative to the field\
    swerve.setDefaultCommand(
        new DriveTeleopCommand(
            driverController::getLeftJoystickX,
            driverController::getLeftJoystickY,
            () -> -driverController.getRightJoystickX(),
            true,
            swerve));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
