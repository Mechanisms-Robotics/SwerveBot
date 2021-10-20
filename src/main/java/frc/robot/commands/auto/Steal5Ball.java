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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.List;
import org.photonvision.PhotonCamera;

public class Steal5Ball extends SequentialCommandGroup {

  private static final SwerveDriveKinematicsConstraint kinematicsConstraint =
      new SwerveDriveKinematicsConstraint(Swerve.kinematics, Swerve.maxVelocity);

  private static final TrajectoryConfig config = new TrajectoryConfig(2.0, 4.0);

  private static final Trajectory trajectory1;
  private static final Trajectory trajectory2;

  static {
    config.addConstraint(kinematicsConstraint);
    trajectory1 =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(90.0)),
            List.of(),
            new Pose2d(new Translation2d(0.0, 3.3), Rotation2d.fromDegrees(90.0)),
            config);

    trajectory2 =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(new Translation2d(0.0, 3.3), Rotation2d.fromDegrees(90.0)),
            List.of(new Translation2d(0.0, 2.2)),
            new Pose2d(new Translation2d(-4.875, 1.0), Rotation2d.fromDegrees(90.0)),
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

  public Steal5Ball(
      Intake intake,
      Hood hood,
      Swerve swerve,
      Shooter shooter,
      Accelerator accelerator,
      Spindexer spindexer,
      PhotonCamera camera) {
    addCommands(
        // Deploy intake and drive trajectory1
        new SwerveControllerCommand(
            trajectory1,
            swerve::getPose,
            swerve.getKinematics(),
            xController,
            yController,
            thetaController,
            () -> Rotation2d.fromDegrees(0.0),
            swerve::setModuleStates,
            swerve).deadlineWith(new IntakeCommand(intake, spindexer, accelerator)),
        // Spinup shooter and spindexer, and drive trajectory2
        new SwerveControllerCommand(
            trajectory2,
            swerve::getPose,
            swerve.getKinematics(),
            xController,
            yController,
            thetaController,
            () -> Rotation2d.fromDegrees(0.0),
            swerve::setModuleStates,
            swerve).deadlineWith(
                    new WaitCommand(1.0).andThen(new SpinupCommand(shooter, accelerator, spindexer))),
        // Aim for 1 seconds
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
            .withTimeout(1.0),
        // Shoot for 1.375 second
        new ShootCommand(shooter, accelerator, spindexer).withTimeout(1.375));
  }
}
