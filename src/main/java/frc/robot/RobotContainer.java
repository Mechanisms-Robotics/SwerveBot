package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ContinuousJogHoodCommand;
import frc.robot.commands.FastShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinupCommand;
import frc.robot.subsystems.*;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer {

  // Subsystems
  // private final Swerve swerve = new Swerve();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final Accelerator accelerator = new Accelerator();
  private final Spindexer spindexer = new Spindexer();
  private final Climber climber = new Climber();

  // The driver's controller
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final Button shootTrigger = new Button(() -> driverController.getRawAxis(3) > 0.1);
  private final Button fastShootButton = new Button(() -> driverController.getAButton());
  private final Button hoodJogForward = new Button(() -> driverController.getBumper(Hand.kRight));
  private final Button hoodJogReverse = new Button(() -> driverController.getBumper(Hand.kLeft));

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    shootTrigger.toggleWhenPressed(
        new SequentialCommandGroup(
            new SpinupCommand(shooter, accelerator, spindexer),
            new ShootCommand(shooter, accelerator, spindexer)));

    fastShootButton.toggleWhenPressed(new FastShootCommand(shooter, accelerator, spindexer));

    hoodJogForward.toggleWhenPressed(new ContinuousJogHoodCommand(hood, false));
    hoodJogReverse.toggleWhenPressed(new ContinuousJogHoodCommand(hood, true));
  }

  private void configureDefaultCommands() {
    // Drive the robot relative to the field\
    /*
    swerve.setDefaultCommand(
        new DriveTeleopCommand(
            () -> driverController.getX(Hand.kLeft),
            () -> -driverController.getY(Hand.kLeft),
            () -> driverController.getX(Hand.kRight),
            swerve));
     */
    // spindexer.setDefaultCommand(
    //        new RunCommand(
    //                () -> spindexer.setOpenLoop(0.15)
    //        )
    // );
    climber.setDefaultCommand(new ClimberCommand(() -> operatorController.getRawAxis(5), climber));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
