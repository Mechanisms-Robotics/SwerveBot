package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;
import frc.robot.commands.auto.Basic3Ball;
import frc.robot.commands.auto.Steal5Ball;
import frc.robot.commands.auto.Trench6Ball;
import frc.robot.commands.auto.Trench8Ball;
import frc.robot.subsystems.*;
import frc.robot.util.PS4Controller;
import frc.robot.util.PS4Controller.Direction;
import org.photonvision.PhotonCamera;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer {

  private static final boolean TUNING_MODE = false;

  // Subsystems
  public final Swerve swerve = new Swerve();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final Accelerator accelerator = new Accelerator();
  private final Spindexer spindexer = new Spindexer();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final PhotonCamera camera = new PhotonCamera("limelight");

  // The driver's controller
  private final PS4Controller driverController = new PS4Controller(0);
  private final PS4Controller secondaryDriverController = new PS4Controller(1);

  // Primary Driver Buttons
  private final Button intakeButton = new Button(driverController::getLeftTriggerButton);
  private final Button intakeToggleButton = new Button(driverController::getLeftBumperButton);
  private final Button aimButton = new Button(driverController::getRightBumperButton);
  private final Button shootButton = new Button(driverController::getRightTriggerButton);
  private final Button climbUpButton = new Button(() -> driverController.getPOV() == Direction.Up);
  private final Button climbDownButton =
      new Button(() -> driverController.getPOV() == Direction.Down);
  private final Button gyroResetButton = new Button(driverController::getShareButton);

  // Secondary Driver Buttons
  private final Button secondaryClimbUpButton =
      new Button(() -> secondaryDriverController.getPOV() == Direction.Up);
  private final Button secondaryClimbDownButton =
      new Button(() -> secondaryDriverController.getPOV() == Direction.Down);

  // Temporary Buttons
  private final Button hoodJogForward =
      new Button(() -> driverController.getPOV() == Direction.Right);
  private final Button hoodJogReverse =
      new Button(() -> driverController.getPOV() == Direction.Left);
  private final Button prepShootButton = new Button(driverController::getCircleButton);
  private final Button justShootButton = new Button(driverController::getSquareButton);

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
    if (TUNING_MODE) {
      CommandScheduler.getInstance().schedule(new PerpetualCommand(new TuneHood(hood, camera)));
    }
  }

  private void configureButtonBindings() {
    // Driver Button Bindings
    intakeButton.toggleWhenPressed(
        new IntakePulseCommand(
            driverController::getLeftBumperButton,
            secondaryDriverController::getXButton,
            secondaryDriverController::getCircleButton,
            intake,
            spindexer,
            accelerator));
    aimButton.toggleWhenPressed(
        new AimCommand(
            driverController::getLeftJoystickX,
            driverController::getLeftJoystickY,
            () -> -driverController.getRightJoystickX(),
            hood,
            swerve,
            shooter,
            accelerator,
            spindexer,
            camera));
    shootButton.whenHeld(new ShootCommand(shooter, accelerator, spindexer));

    climbUpButton.whenHeld(
        new StartEndCommand(() -> climber.setOpenLoop(0.20), climber::stop, climber));
    climbDownButton.whenHeld(
        new StartEndCommand(() -> climber.setOpenLoop(-0.20), climber::stop, climber));

    secondaryClimbUpButton.whenHeld(
        new StartEndCommand(() -> climber.setOpenLoop(0.75), climber::stop, climber));
    secondaryClimbDownButton.whenHeld(
        new StartEndCommand(() -> climber.setOpenLoop(-0.75), climber::stop, climber));

    gyroResetButton.whenPressed(new InstantCommand(swerve::zeroHeading));

    // Temporary Button Bindings
    hoodJogForward.whenHeld(new ContinuousJogHoodCommand(hood, false));
    hoodJogReverse.whenHeld(new ContinuousJogHoodCommand(hood, true));
    prepShootButton.whenPressed(new PrepShootCommand(spindexer, accelerator));
    justShootButton.toggleWhenPressed(
        new SequentialCommandGroup(
            new SpinupCommand(spindexer, accelerator, shooter, camera),
            new ShootCommand(shooter, accelerator, spindexer)));
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

    climber.setDefaultCommand(new RunCommand(climber::stop, climber));
  }

  public Command getAutonomousCommand() {
    // return new RunCommand(() -> swerve.drive(0.0, 1.0, 0.0, false), swerve).withTimeout(2.0);
   // return new Basic3Ball(hood, swerve, shooter, accelerator, spindexer, camera);
    return new Steal5Ball(intake, hood, swerve, shooter, accelerator, spindexer, camera);
  }
}
