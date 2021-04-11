package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveTrajectory {
    
    private final Trajectory xyTrajectory;
    private final TrapezoidProfile[] rotationTrajectory;

    public SwerveTrajectory(Trajectory xyTrajectory, TrapezoidProfile[] rotationTrajectory) {
        this.xyTrajectory = xyTrajectory;
        this.rotationTrajectory = rotationTrajectory;
    }

    public class TrajectoryPoint {
        public Translation2d xyLocation;
        public Rotation2d targetRotation;
    }

    public static SwerveTrajectory generateTrajectory(TrajectoryPoint[] points, TrajectoryConfig xyConfig, TrapezoidProfile.Constraints rotationConfig) {
        if (points)
    }
}
