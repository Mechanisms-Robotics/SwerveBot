package frc.robot.util;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class RotationTrajectory {

    private final TrapezoidProfile[] trajectory;
    private final TrajectoryConfig config;

    private RotationTrajectory(Rotation)
    
    public class Config {
        public TrapezoidProfile.Constraints constraints;
        public double startVelocity, endVelocity;

        public TrajectoryConfig(double maxVelocity, double maxAcceleration) {
            constraints.maxVelocity = maxVelocity;
            constraints.maxAcceleration = maxAcceleration;
            startVelocity = 0.0;
            endVelocity = 0.0;
        }
    }
}
