package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.subsystems.Swerve;

import java.util.List;

public class TestYMovement extends SequentialCommandGroup {

    private static final SwerveDriveKinematicsConstraint kinematicsConstraint =
        new SwerveDriveKinematicsConstraint(
                Swerve.kinematics,
                Swerve.maxVelocity
    );


    private static final TrajectoryConfig config = new TrajectoryConfig(
            1.5,
            3.0
    );

    private static final Trajectory trajectory;

    static {
        config.addConstraint(kinematicsConstraint);
        trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(90.0)),
                List.of(),
                new Pose2d(new Translation2d(1.0, 3.0), Rotation2d.fromDegrees(90.0)),
                config
        );
    }

    public TestYMovement(Swerve swerve) {
        addCommands(new DriveTrajectoryCommand(trajectory, swerve));
    }
}
