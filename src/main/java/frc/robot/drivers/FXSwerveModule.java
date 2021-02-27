package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * A hardware wrapper class for a swerve module.
 */
public class FXSwerveModule implements SwerveModule {

  private final WPI_TalonFX wheelMotor;
  private final WPI_TalonFX steerMotor;
  private final CANCoder steeringEncoder;

  private static final CANCoderConfiguration canCoderConfig;

  private static int speedEncoderCPR = 2048; // counts per revolution
  private static final double moduleGearRatio = 8.61; // : 1
  private static final double wheelDiameter = 0.0762; // m
  private static final double ticksPerSecToMeterPerSec
      = (wheelDiameter * Math.PI)
      / (double)  (speedEncoderCPR * moduleGearRatio);

  private static final double speedP = 0.0;
  private static final double speedI = 0.0;
  private static final double speedD = 0.0;
  private static final int speedPIDSlot = 0;

  private static final double rotationP = 0.0;
  private static final double rotationI = 0.0;
  private static final double rotationD = 0.0;
  private static final double rotationMaxSpeed = 0.0; // rad / sec
  private static final double rotationMaxAccel = 0.0;

  private static final ProfiledPIDController steeringAnglePID = new ProfiledPIDController(
      rotationP, rotationI, rotationD,
      new TrapezoidProfile.Constraints(
          rotationMaxSpeed,
          rotationMaxAccel
    )
  );

  /**
   * Constructs a swerve module.
   *
   * @param steerID          The CAN ID of the Spark Max used to control the steering
   * @param wheelID          The CAN Id of the Spark Max used to control the wheel
   * @param encoderID        The CAN ID of the CANCoder used to determine the angle of the module
   * @param steeringReversed Wether or not the motor used for steering is reversed
   * @param wheelReversed    Wether or not the motor used for the wheel is reversed
   */
  public FXSwerveModule(int steerID, int wheelID, int encoderID,
      boolean steeringReversed, boolean wheelReversed) {
    wheelMotor = new WPI_TalonFX(wheelID);
    wheelMotor.setInverted(wheelReversed);

    steerMotor = new WPI_TalonFX(steerID);
    steerMotor.setInverted(steeringReversed);

    wheelMotor.setSelectedSensorPosition(0.0);
    wheelMotor.config_kP(speedPIDSlot, speedP);
    wheelMotor.config_kI(speedPIDSlot, speedI);
    wheelMotor.config_kD(speedPIDSlot, speedD);

    steeringEncoder = new CANCoder(encoderID);
    steeringEncoder.configAllSettings(canCoderConfig);
  }

  /**
   * Constructs a swerve module.
   *
   * @param steeringID The CAN ID of the Spark Max used to control the steering
   * @param wheelID    The CAN Id of the Spark Max used to control the wheel
   * @param encoderID  The CAN ID of the CANCoder used to determine the angle of the module
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