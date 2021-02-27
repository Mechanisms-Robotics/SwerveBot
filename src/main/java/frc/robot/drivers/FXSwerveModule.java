package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * A hardware wrapper class for a swerve module.
 */
public class FxSwerveModule implements SwerveModule {
    
  private final WPI_TalonFX wheelMotor;
  private final WPI_TalonFX steerMotor;
  
  /**
   * Constructs a swerve modules.
   *
   * @param steerId The CAN ID of the Spark Max used to control the steering
   * @param wheelId The CAN Id of the Spark Max used to control the wheel
   * @param encoderId The CAN ID of the CANCoder used to determine the angle of the module
   * @param steeringReversed Wether or not the motor used for steering is reversed
   * @param wheelReversed Wether or not the motor used for the wheel is reversed
   */
  public FxSwerveModule(int steerId, int wheelId, int encoderId,
        boolean steeringReversed, boolean wheelReversed) {
    steerMotor = new WPI_TalonFX(steerId);
    wheelMotor = new WPI_TalonFX(wheelId);
  }

  /**
   * Constructs a swerve modules.
   *
   * @param steerId The CAN ID of the Spark Max used to control the steering
   * @param wheelId The CAN Id of the Spark Max used to control the wheel
   * @param encoderId The CAN ID of the CANCoder used to determine the angle of the module
   */
  public FxSwerveModule(int steerId, int wheelId, int encoderId) {
    this(steerId, wheelId, encoderId, false, false);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState();
  }

  @Override
  public void setState(SwerveModuleState state) {
    
  }
}