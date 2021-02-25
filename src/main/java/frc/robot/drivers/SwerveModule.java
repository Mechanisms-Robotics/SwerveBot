package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * A hardware wrapper class for a swerve module
 */
public class SwerveModule
{   
    
    private final TalonFX wheelMotor_;
    private final TalonFX steerMotor_;
    
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
    public SwerveModule(int steerID, int wheelID, int encoderId,
        boolean steeringReversed, boolean wheelReversed)
    {
        wheelMotor_ = new TalonFX(wheelID);
        steerMotor_ = new TalonFX(steerID);
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
    public SwerveModule(int steeringID, int wheelID, int encoderID)
    {
        this(steeringID, wheelID, encoderID, false, false);
    }

    /**
     * Get the current state of the module
     * @return A SwerveModuleState representing the current speed
     *  and rotation of the module
     */
    public SwerveModuleState getState()
    {
        return new SwerveModuleState();
    }

    /**
     * Set the current state of the swerve module
     * @param state The SwerveModuleState to set the module to
     */
    public void setState(SwerveModuleState state)
    {
        
    }

    /**
     * Reset the encoders to 0
     */
    public void resetEncoders()
    {
    }
}
