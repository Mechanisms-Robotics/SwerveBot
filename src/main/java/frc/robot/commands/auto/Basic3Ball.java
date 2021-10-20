package frc.robot.commands.auto;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.List;
import org.photonvision.PhotonCamera;

public class Basic3Ball extends SequentialCommandGroup {

  private static final SwerveDriveKinematicsConstraint kinematicsConstraint =
      new SwerveDriveKinematicsConstraint(Swerve.kinematics, Swerve.maxVelocity);

  private static final TrajectoryConfig config = new TrajectoryConfig(2.0, 4.0);

  private static final Trajectory trajectory;

  static {
    config.addConstraint(kinematicsConstraint);
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(90.0)),
            List.of(),
            new Pose2d(new Translation2d(0.0, -1.0), Rotation2d.fromDegrees(90.0)),
            config);
  }

  private final PIDController xController = new PIDController(linearGain, 0.0, 0.0, loopTime);
  private final PIDController yController = new PIDController(linearGain, 0.0, 0.0, loopTime);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          headingGain,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(headingMaxVelocity, headingMaxVelocity),
          loopTime);

  public Basic3Ball(
      Hood hood,
      Swerve swerve,
      Shooter shooter,
      Accelerator accelerator,
      Spindexer spindexer,
      PhotonCamera camera) {
    addCommands(
        new AimCommand(
                () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                hood,
                swerve,
                shooter,
                accelerator,
                spindexer,
                camera)
            .withTimeout(2.0),
        new ShootCommand(shooter, accelerator, spindexer).withTimeout(1.0),
        new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            xController,
            yController,
            thetaController,
            () -> Rotation2d.fromDegrees(0.0),
            swerve::setModuleStates,
            swerve));
  }
}
