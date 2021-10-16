package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** This command is used for driving a trajectories during autonomous. */
public class DriveTrajectoryCommand extends SwerveControllerCommand {

  // A bit of a convention break. Normally you would not make something like a controller a static
  // variable. This is because if there were multiple instances of the command running then they
  // would all be updating the same control with would cause issues. However, in this case I didn't
  // want to recreate the large controller class every time a new trajectory is run.
  // I also know that because this command requires a system the scheduler will
  // only ever run one DriveTrajectoryCom/home/alexomand at once. So this is 'safe' in this
  // instance.

  private static final double xGain = 0.1;
  private static final double yGain = 0.1;
  private static final double thetaGain = 2.0;
  private static final double maxThetaVelocity = Math.PI; // radians per second
  private static final double maxThetaAcceleration = Math.PI;

  private static final PIDController xController =
      new PIDController(xGain, 0.0, 0.0, Constants.loopTime);
  private static final PIDController yController =
      new PIDController(yGain, 0.0, 0.0, Constants.loopTime);
  private static final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaGain,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(maxThetaVelocity, maxThetaAcceleration),
          Constants.loopTime);

  static {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Drive a given trajectory. The heading along the trajectory will be the heading of the robot at
   * the final pose in the trajectory.
   *
   * @param xyTrajectory The trajectory that the center of the robot will follow. Note that the
   *     heading at the end of this trajectory defines the heading of the robot along the entire
   *     trajectory.
   * @param swerve An instance of a swerve drive.
   */
  public DriveTrajectoryCommand(Trajectory xyTrajectory, Swerve swerve) {
    super(
        xyTrajectory,
        swerve::getPose,
        swerve.getKinematics(),
        xController,
        yController,
        thetaController,
        DriveTrajectoryCommand::getRotation,
        swerve::setModuleStates,
        swerve);
  }

  private static Rotation2d getRotation() {
    return new Rotation2d(0.0);
  }
}
