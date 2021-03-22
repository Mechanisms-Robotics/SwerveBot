package frc.robot.drivers;

import static frc.robot.Constants.loopTime;
import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import java.lang.Math;

/**
 * A hardware wrapper class for a swerve module that uses Falcon500s.
 */
public class FxSwerveModule implements SwerveModule {

  private static int falconCPR = 2048; // counts per revolution
  private static final double moduleGearRatio = 6.86; // : 1
  private static final double wheelDiameter = 0.1016; // m
  private static final double ticksPer100msToMeterPerSec
      = ((wheelDiameter * Math.PI)
      / (double)  (falconCPR * moduleGearRatio)) * 10.0;
  private static final double steerGearRatio = 12.8; // : 1
  private static final double ticksToDegrees = 360.0 / (falconCPR * steerGearRatio);

  private static final double speedP = 0.0;
  private static final double speedI = 0.0;
  private static final double speedD = 0.0;
  private static final int speedPIDSlot = 0;

  private static final double steerP = 0.0;
  private static final double steerI = 0.0;
  private static final double steerD = 0.0;
  private static final int steerPIDSlot = 0;

  private static final int uniqueId = 1602616989;
  private static final int customParamIdx = 0;

  private final WPI_TalonFX wheelMotor;
  private final WPI_TalonFX steerMotor;
  private final CANCoder steeringEncoder;
  private final String moduleName;

  /**
   * Constructs a swerve module.
   *
   * @param steerId          The CAN ID of the Spark Max used to control the steering
   * @param wheelId          The CAN Id of the Spark Max used to control the wheel
   * @param encoderId        The CAN ID of the CANCoder used to determine the angle of the module
   * @param steeringReversed Wether or not the motor used for steering is reversed
   * @param wheelReversed    Wether or not the motor used for the wheel is reversed
   */
  public FxSwerveModule(String name, int steerId, int wheelId,
      int encoderId, boolean steeringReversed, boolean wheelReversed) {

    moduleName = name;

    wheelMotor = new WPI_TalonFX(wheelId);
    wheelMotor.configFactoryDefault(startupCanTimeout);
    wheelMotor.setInverted(wheelReversed);
    wheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, startupCanTimeout);

    steerMotor = new WPI_TalonFX(steerId);
    steerMotor.configFactoryDefault(startupCanTimeout);
    steerMotor.setInverted(steeringReversed);

    wheelMotor.config_kP(speedPIDSlot, speedP, startupCanTimeout);
    wheelMotor.config_kI(speedPIDSlot, speedI, startupCanTimeout);
    wheelMotor.config_kD(speedPIDSlot, speedD, startupCanTimeout);

    steeringEncoder = new CANCoder(encoderId);
    bootCanCoder();

    steerMotor.configSelectedFeedbackCoefficient(ticksToDegrees, steerPIDSlot, startupCanTimeout);
    steerMotor.setSelectedSensorPosition(steeringEncoder.getAbsolutePosition(), steerPIDSlot,
        startupCanTimeout);
    steerMotor.config_kP(steerPIDSlot, steerP, startupCanTimeout);
    steerMotor.config_kI(steerPIDSlot, steerI, startupCanTimeout);
    steerMotor.config_kD(steerPIDSlot, steerD, startupCanTimeout);
  }

  /**
   * Constructs a swerve module.
   *
   * @param name Unique name for this swerve module.
   * @param steeringId The CAN ID of the Spark Max used to control the steering
   * @param wheelId    The CAN Id of the Spark Max used to control the wheel
   * @param encoderId  The CAN ID of the CANCoder used to determine the angle of the module
   */
  public FxSwerveModule(String name, int steeringId, int wheelId, int encoderId) {
    this(name, steeringId, wheelId, encoderId, false, false);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        wheelMotor.getSelectedSensorVelocity() * ticksPer100msToMeterPerSec,
        Rotation2d.fromDegrees(steerMotor.getSelectedSensorPosition()));
  }

  @Override
  public void setState(SwerveModuleState state) {
    wheelMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / ticksPer100msToMeterPerSec);
    steerMotor.set(ControlMode.MotionMagic, unwrapAngle(state.angle.getDegrees()));
  }

  @Override
  public void calibrateAbsoluteEncoder() {
    steeringEncoder.configFactoryDefault(startupCanTimeout);
    steeringEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition, startupCanTimeout);
    steeringEncoder.configMagnetOffset(-steeringEncoder.getAbsolutePosition(), startupCanTimeout);
    steeringEncoder.setPositionToAbsolute(startupCanTimeout);
    steeringEncoder.configSetCustomParam(uniqueId, customParamIdx, startupCanTimeout);
    bootCanCoder();
  }

  private void bootCanCoder() {
    steeringEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, loopTime,
        startupCanTimeout);
    steeringEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180,
        startupCanTimeout);
    if (steeringEncoder.configGetCustomParam(customParamIdx, startupCanTimeout) != uniqueId) {
      DriverStation.reportError("Swerve module[" + moduleName + "] needs to be recalibrate", false);
    }
  }

  private double unwrapAngle(double angle) {
    int turns = (int) (steerMotor.getSelectedSensorPosition() / 360.0);
    return angle + (turns * 360);
  }
}
