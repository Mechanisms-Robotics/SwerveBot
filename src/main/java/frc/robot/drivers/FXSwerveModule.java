package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import java.lang.Math;

/**
 * A hardware wrapper class for a swerve module.
 */
public class FXSwerveModule implements SwerveModule {
  private final WPI_TalonFX wheelMotor;
  private final WPI_TalonFX steerMotor;
  private final CANCoder steeringEncoder;

  private static final CANCoderConfiguration canCoderConfig;

  private static int speedEncoderCPR = 2048; // counts per revolution
  private static final double moduleGearRatio = 6.86; // : 1
  private static final double wheelDiameter = 0.0762; // m
  private static final double ticksPer100msToMeterPerSec
      = ((wheelDiameter * Math.PI)
      / (double)  (speedEncoderCPR * moduleGearRatio)) * 10.0;
  private static final double steerEncoderCPR = 2048;
  private static final double steerGearRatio = 12.8; // : 1
  private static final double ticksToDegrees = 360.0 / (steerEncoderCPR * steerGearRatio);

  private static final double speedP = 0.0;
  private static final double speedI = 0.0;
  private static final double speedD = 0.0;
  private static final int speedPIDSlot = 0;

  private static final double steerP = 0.0;
  private static final double steerI = 0.0;
  private static final double steerD = 0.0;
  private static final int steerPIDSlot = 0;

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
    wheelMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    wheelMotor = new WPI_TalonFX(wheelID);
    wheelMotor.setInverted(wheelReversed);
    wheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);

    steerMotor = new WPI_TalonFX(steerID);
    steerMotor.setInverted(steeringReversed);

    wheelMotor.setSelectedSensorPosition(0.0);
    wheelMotor.config_kP(speedPIDSlot, speedP);
    wheelMotor.config_kI(speedPIDSlot, speedI);
    wheelMotor.config_kD(speedPIDSlot, speedD);

    steeringEncoder = new CANCoder(encoderID);
    steeringEncoder.configAllSettings(canCoderConfig);
    steerMotor.configSelectedFeedbackCoefficient(ticksToDegrees, steerPIDSlot, 100);
    steerMotor.setSelectedSensorPosition(steeringEncoder.getAbsolutePosition());
    steerMotor.config_kP(steerPIDSlot, steerP);
    steerMotor.config_kI(steerPIDSlot, steerI);
    steerMotor.config_kD(steerPIDSlot, steerD);
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
    return new SwerveModuleState(wheelMotor.getSelectedSensorVelocity() * ticksPer100msToMeterPerSec,
        Rotation2d.fromDegrees(wrapAngle(steerMotor.getSelectedSensorPosition())));
  }

  @Override
  public void setState(SwerveModuleState state) {
    wheelMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / ticksPer100msToMeterPerSec);
    steerMotor.set(ControlMode.MotionMagic, unwrapAngle(state.angle.getDegrees()));
  }

  public double wrapAngle(double angle) {
    return angle % (360 * Math.signum(angle));
  }

  public double unwrapAngle(double angle) {
    int turns = (int)(steerMotor.getSelectedSensorPosition() / 360.0);
    return angle + (turns * 360);
  }
}
