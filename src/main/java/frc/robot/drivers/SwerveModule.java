package frc.robot.drivers;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * This interface defines the api needed to control a swerve module fully.
 */
public interface SwerveModule {

  /**
   * Get the current state of the module.
   *
   * @return A SwerveModuleState representing the current speed
   *     and rotation of the module
   */
  public SwerveModuleState getState();

  /**
   * Set the current state of the swerve module.
   *
   * @param state The SwerveModuleState to set the module to
   */
  public void setState(SwerveModuleState state);

  /**
   * Sets the wheel and steering speed of the module.
   *
   * @param wheelSpeed The speed in m/s to set the wheel speed to.
   * @param steeringSpeed The speed in rads/s to s
   */
  public void setVelocity(double wheelSpeed, double steeringSpeed);

  /**
   * Get the rotational speed of the modules.
   *
   * @return The modules steering speed in rads/s
   */
  public double getSteeringSpeed();

  /**
   * Calibrate the absolute encoder.
   */
  public void calibrateAbsoluteEncoder();
}
