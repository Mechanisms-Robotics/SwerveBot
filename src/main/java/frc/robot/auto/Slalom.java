package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;

public class Slalom extends SequentialCommandGroup {
  private Trajectory slalomTrajectory;

  private final TrajectoryConfig config = new TrajectoryConfig(Swerve.maxVelocity * 0.9, 3.0);

  public Slalom(Swerve swerve) {
    this.generateTrajectories();

    swerve.resetController();
    addCommands(
        new DriveTrajectoryCommand(slalomTrajectory, swerve).andThen(swerve::resetController));
  }

  public void generateTrajectories() {
    {
      ArrayList<Pose2d> points = new ArrayList<>();

      points.add(new Pose2d(0.762, 0, new Rotation2d()));
      points.add(new Pose2d(1.524, 0.762, new Rotation2d()));
      points.add(new Pose2d(2.286, 1.524, new Rotation2d()));
      points.add(new Pose2d(5.334, 1.524, new Rotation2d()));
      points.add(new Pose2d(6.858, 0, new Rotation2d()));
      points.add(new Pose2d(7.62, 0.762, new Rotation2d()));
      points.add(new Pose2d(6.858, 1.524, new Rotation2d()));
      points.add(new Pose2d(5.334, 0, new Rotation2d()));
      points.add(new Pose2d(2.286, 0, new Rotation2d()));
      points.add(new Pose2d(1.524, 0.762, new Rotation2d()));
      points.add(new Pose2d(0.762, 1.524, new Rotation2d()));
      points.add(new Pose2d(0, 1.524, new Rotation2d()));

      this.slalomTrajectory = TrajectoryGenerator.generateTrajectory(points, config);
    }
  }
}
