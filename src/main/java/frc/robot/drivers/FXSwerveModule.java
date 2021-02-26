package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * A hardware wrapper class for a swerve module
 */
public class FXSwerveModule implements SwerveModule {   
    
    private final WPI_TalonFX wheelMotor_;
    private final WPI_TalonFX steerMotor_;
    
    /**
     * Constructs a swerve modules
     * @param steerID 
     *  The CAN ID of the Spark Max used to control the steering
     * @param wheelID
     *  The CAN Id of the Spark Max used to control the wheel
     * @param encoderId
     *  The CAN ID of the CANCoder used to determine the angle of the module
     * @param steeringReversed
     *  Wether or not the motor used for steering is reversed
     * @param wheelReversed
     *  Wether or not the motor used for the wheel is reversed
     */
    public FXSwerveModule(int steerID, int wheelID, int encoderId,
        boolean steeringReversed, boolean wheelReversed) {
        wheelMotor_ = new WPI_TalonFX(wheelID);
        steerMotor_ = new WPI_TalonFX(steerID);
    }

    /**
     * Constructs a swerve modules
     * @param steeringID 
     *  The CAN ID of the Spark Max used to control the steering
     * @param wheelID
     *  The CAN Id of the Spark Max used to control the wheel
     * @param encoderId
     *  The CAN ID of the CANCoder used to determine the angle of the module
     */
    public FXSwerveModule(int steeringID, int wheelID, int encoderID) {
        this(steeringID, wheelID, encoderID, false, false);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    @Override
    public void setState(SwerveModuleState state) {
        
    }
}