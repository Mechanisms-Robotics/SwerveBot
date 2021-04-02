package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;

public class Slalom extends AutoCommand {
  private Trajectory slalomTrajectory;

  private final Swerve swerve;

  private final TrajectoryConfig config = new TrajectoryConfig(Swerve.maxVelocity * 0.9, 3.0);

  public Slalom(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
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

  @Override
  public void initialize() {
    if (this.slalomTrajectory == null) {
      this.generateTrajectories();
    }
  }

  @Override
  public void execute() {
    new DriveTrajectoryCommand(slalomTrajectory, this.swerve);
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.resetController();
  }
}
