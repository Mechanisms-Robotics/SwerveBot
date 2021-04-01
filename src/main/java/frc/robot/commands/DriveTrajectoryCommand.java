package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Swerve;

public class DriveTrajectoryCommand extends CommandBase {

  private static final double maxTranslationalVelocity = Swerve.maxVelocity * 0.90;
  private static final double maxRotationalVelocity = Swerve.maxRotationalVelocity;

  private static final double kPX = 0.0;
  private static final double kIX = 0.0;
  private static final double kDX = 0.0;

  private static final double kPY = 0.0;
  private static final double kIY = 0.0;
  private static final double kDY = 0.0;

  private static final double kPTheta = 0.0;
  private static final double kITheta = 0.0;
  private static final double kDTheta = 0.0;

  private final Trajectory trajectory;
  private final Swerve swerve;

  public DriveTrajectoryCommand(Trajectory trajectory, Swerve swerve) {
    this.trajectory = trajectory;
    this.swerve = swerve;
  }

  @Override
  public void execute() {
    ProfiledPIDController thetaPIDController =
        new ProfiledPIDController(
            kPTheta,
            kITheta,
            kDTheta,
            new Constraints(maxTranslationalVelocity, maxRotationalVelocity));

    thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            new PIDController(kPX, kIX, kDX),
            new PIDController(kPY, kIY, kDY),
            thetaPIDController,
            swerve::setModuleStates,
            swerve);

    swerve.resetController();
    
    swerveControllerCommand.andThen(() -> swerve.driveOpenLoop(0, 0, 0, false));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.resetController();
  }
}
