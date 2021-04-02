package frc.robot.subsystems;

import static frc.robot.Constants.loopTime;
import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.Constants.talonPrimaryPid;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/** A hardware wrapper class for a swerve module that uses Falcon500s. */
public class FxSwerveModule implements SwerveModule {

  private static int falconCPR = 2048; // counts per revolution
  private static final double moduleGearRatio = 6.86; // : 1
  private static final double wheelDiameter = 0.1016; // m
  private static final double ticksPer100msToMeterPerSec =
      ((wheelDiameter * Math.PI) / (double) (falconCPR * moduleGearRatio)) * 10.0;
  private static final double steerGearRatio = 12.8; // : 1
  private static final double degreesToTicks = (falconCPR * steerGearRatio) / 360;

  private static final double wheelSpeedP = 0.0;
  private static final double wheelSpeedI = 0.0;
  private static final double wheelSpeedD = 0.0;
  private static final int speedPIDSlot = 0;

  private static final double steerMotionP = 0.0;
  private static final double steerMotionI = 0.0;
  private static final double steerMotionD = 0.0;
  private static final int steerMotionSlot = 0;

  private static final double steerSpeedP = 0.0;
  private static final double steerSpeedI = 0.0;
  private static final double steerSpeedD = 0.0;
  private static final int steerSpeedSlot = 1;

  private static final int uniqueId = 1602616989;
  private static final int customParamIdx = 0;

  private static final double currentLimitWheel = 30.0; // Amps
  private static final double currentLimitSteer = 10.0; // Amps
  private static final double currentTriggerWheel = 35.0; // Amps
  private static final double currentTriggerSteer = 15; // Amps
  private static final double currentTimeWheel = 0.5; // Secs
  private static final double currentTimeSteer = 0.1; // Secs

  private final WPI_TalonFX wheelMotor;
  private final WPI_TalonFX steerMotor;
  private final CANCoder steeringEncoder;
  private final String moduleName;

  private boolean velocityMode = false;

  /**
   * Constructs a swerve module.
   *
   * @param steerId The CAN ID of the Spark Max used to control the steering
   * @param wheelId The CAN Id of the Spark Max used to control the wheel
   * @param encoderId The CAN ID of the CANCoder used to determine the angle of the module
   * @param steeringReversed Wether or not the motor used for steering is reversed
   * @param wheelReversed Wether or not the motor used for the wheel is reversed
   */
  public FxSwerveModule(
      String name,
      int steerId,
      int wheelId,
      int encoderId,
      boolean steeringReversed,
      boolean wheelReversed) {

    moduleName = name;

    wheelMotor = new WPI_TalonFX(wheelId);
    wheelMotor.configFactoryDefault(startupCanTimeout);
    wheelMotor.setInverted(TalonFXInvertType.CounterClockwise);
    wheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, startupCanTimeout);
    wheelMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, talonPrimaryPid, startupCanTimeout);
    wheelMotor.config_kP(speedPIDSlot, wheelSpeedP, startupCanTimeout);
    wheelMotor.config_kI(speedPIDSlot, wheelSpeedI, startupCanTimeout);
    wheelMotor.config_kD(speedPIDSlot, wheelSpeedD, startupCanTimeout);
    // TODO: Replace with actual values
    wheelMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, currentLimitWheel, currentTriggerWheel, currentTimeWheel));

    steeringEncoder = new CANCoder(encoderId);
    bootCanCoder();

    steerMotor = new WPI_TalonFX(steerId);
    steerMotor.configFactoryDefault(startupCanTimeout);
    steerMotor.setInverted(TalonFXInvertType.Clockwise);
    steerMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, talonPrimaryPid, startupCanTimeout);
    steerMotor.setSelectedSensorPosition(
        steeringEncoder.getAbsolutePosition(), talonPrimaryPid, startupCanTimeout);
    steerMotor.config_kP(steerMotionSlot, steerMotionP, startupCanTimeout);
    steerMotor.config_kI(steerMotionSlot, steerMotionI, startupCanTimeout);
    steerMotor.config_kD(steerMotionSlot, steerMotionD, startupCanTimeout);
    steerMotor.config_kP(steerSpeedSlot, steerSpeedP, startupCanTimeout);
    steerMotor.config_kI(steerSpeedSlot, steerSpeedI, startupCanTimeout);
    steerMotor.config_kD(steerSpeedSlot, steerSpeedD, startupCanTimeout);
    // TODO: Replace with actual values
    steerMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, currentLimitSteer, currentTriggerSteer, currentTimeSteer));
  }

  /**
   * Constructs a swerve module.
   *
   * @param name Unique name for this swerve module.
   * @param steeringId The CAN ID of the Spark Max used to control the steering
   * @param wheelId The CAN Id of the Spark Max used to control the wheel
   * @param encoderId The CAN ID of the CANCoder used to determine the angle of the module
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
    if (velocityMode) {
      velocityMode = false;
      steerMotor.selectProfileSlot(steerMotionSlot, talonPrimaryPid);
    }
    wheelMotor.set(
        TalonFXControlMode.Velocity, state.speedMetersPerSecond / ticksPer100msToMeterPerSec);
    steerMotor.set(
        TalonFXControlMode.MotionMagic, unwrapAngle(state.angle.getDegrees()) * degreesToTicks);
  }

  @Override
  public void setVelocity(double wheelSpeed, double steeringSpeed) {
    if (!velocityMode) {
      velocityMode = true;
      steerMotor.selectProfileSlot(steerSpeedSlot, talonPrimaryPid);
    }
    wheelMotor.set(TalonFXControlMode.Velocity, wheelSpeed / ticksPer100msToMeterPerSec);
    steerMotor.set(
        TalonFXControlMode.Velocity, Math.toDegrees(steeringSpeed) * degreesToTicks * 10);
  }

  @Override
  public double getSteeringSpeed() {
    return Math.toRadians(steeringEncoder.getVelocity());
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
    steeringEncoder.setStatusFramePeriod(
        CANCoderStatusFrame.SensorData, (int) (loopTime * 1000), startupCanTimeout);
    steeringEncoder.configAbsoluteSensorRange(
        AbsoluteSensorRange.Signed_PlusMinus180, startupCanTimeout);
    if (steeringEncoder.configGetCustomParam(customParamIdx, startupCanTimeout) != uniqueId) {
      DriverStation.reportError("Swerve module[" + moduleName + "] needs to be recalibrate", false);
    }
  }

  private double unwrapAngle(double angle) {
    int turns = (int) (steerMotor.getSelectedSensorPosition() / 360.0);
    return angle + (turns * 360);
  }
}
