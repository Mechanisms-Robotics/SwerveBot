package frc.robot.drivers;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public interface SwerveModule {

    /**
     * Get the current state of the module
     * @return A SwerveModuleState representing the current speed
     *  and rotation of the module
     */
    public SwerveModuleState getState();

    /**
     * Set the current state of the swerve module
     * @param state The SwerveModuleState to set the module to
     */
    public void setState(SwerveModuleState state);
}
