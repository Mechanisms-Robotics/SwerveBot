package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDriveTrajectory extends SwerveControllerCommand {

  private static final PIDController xController =
      new PIDController(0.0, 0.0, 0.0, Constants.loopTime);
  private static final PIDController yController =
      new PIDController(0.0, 0.0, 0.0, Constants.loopTime);
  private static final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI),
          Constants.loopTime);

  public SwerveDriveTrajectory(Trajectory trajectory, Swerve swerve) {
    super(
        trajectory,
        swerve::getPose,
        swerve.getKinematics(),
        xController,
        yController,
        thetaController,
        swerve::setModuleStates,
        swerve);
  }
}
