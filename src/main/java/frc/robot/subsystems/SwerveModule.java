package frc.robot.subsystems;

import static frc.robot.Constants.falconCPR;
import static frc.robot.Constants.loopTime;
import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.Constants.talonPrimaryPid;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** A hardware wrapper class for a swerve module that uses Falcon500s. */
public class SwerveModule implements Loggable {

  private static final double moduleGearRatio = 6.86; // : 1
  private static final double wheelDiameter = 0.1016; // meters
  private static final double ticksPer100msToMeterPerSec =
      ((wheelDiameter * Math.PI) / (double) (falconCPR * moduleGearRatio)) * 10.0;
  private static final double steerGearRatio = 12.8; // : 1
  private static final double degreesToTicks = (falconCPR * steerGearRatio) / 360;
  private static final int uniqueId = 5123457;
  private static final int velocityPidSlot = 0;
  private static final int motionMagicPidSlot = 1;

  private static final TalonFXConfiguration wheelMotorConfig = new TalonFXConfiguration();
  private static final TalonFXConfiguration steeringMotorConfig = new TalonFXConfiguration();
  private static final CANCoderConfiguration angleEncoderConfig = new CANCoderConfiguration();

  static {
    // Wheel Motor Current Limiting
    var wheelMotorCurrentLimit = new SupplyCurrentLimitConfiguration();
    wheelMotorCurrentLimit.enable = true;
    wheelMotorCurrentLimit.currentLimit = 30; // Amps
    wheelMotorCurrentLimit.triggerThresholdCurrent = 35; // Amps
    wheelMotorCurrentLimit.triggerThresholdTime = 0.5; // Seconds
    wheelMotorConfig.supplyCurrLimit = wheelMotorCurrentLimit;

    // Configure wheel motor current limiting
    wheelMotorConfig.voltageCompSaturation = 12.0; // Volts

    // Configure steering motor current limiting
    var steeringMotorCurrentLimit = new SupplyCurrentLimitConfiguration();
    steeringMotorCurrentLimit.enable = true;
    steeringMotorCurrentLimit.currentLimit = 15; // Amps
    steeringMotorCurrentLimit.triggerThresholdCurrent = 25; // Amps
    steeringMotorCurrentLimit.triggerThresholdTime = 0.5; // Seconds
    steeringMotorConfig.supplyCurrLimit = steeringMotorCurrentLimit;

    steeringMotorConfig.voltageCompSaturation = 8.0; // Volts

    angleEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    angleEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
  }

  private final WPI_TalonFX wheelMotor;
  private final WPI_TalonFX steerMotor;
  private final CANCoder steeringEncoder;
  private final String moduleName;

  /**
   * Constructs a swerve module.
   *
   * @param name The unique name of this swerve module
   * @param wheelMotorId The CAN ID of the Spark Max used to control the wheel
   * @param steeringMotorId The CAN Id of the Spark Max used to control the steering
   * @param angleEncoderId The CAN ID of the CANCoder used to determine the angle of the module
   */
  public SwerveModule(String name, int wheelMotorId, int steeringMotorId, int angleEncoderId) {

    moduleName = name;

    // Setup wheel motor
    wheelMotor = new WPI_TalonFX(wheelMotorId);
    wheelMotor.configAllSettings(wheelMotorConfig, startupCanTimeout);
    wheelMotor.setInverted(TalonFXInvertType.CounterClockwise);
    wheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, startupCanTimeout);

    // Configure wheel motor PID
    wheelMotor.config_kP(velocityPidSlot, 0.01, startupCanTimeout);
    wheelMotor.config_kI(velocityPidSlot, 0.0, startupCanTimeout);
    wheelMotor.config_kD(velocityPidSlot, 0.0, startupCanTimeout);

    // Setup steering encoder
    steeringEncoder = new CANCoder(angleEncoderId);
    bootCanCoder();

    // Setup steering motor
    steerMotor = new WPI_TalonFX(steeringMotorId);
    steerMotor.configAllSettings(steeringMotorConfig, startupCanTimeout);
    steerMotor.setInverted(TalonFXInvertType.Clockwise);
    steerMotor.setSelectedSensorPosition(
        steeringEncoder.getAbsolutePosition() * degreesToTicks, talonPrimaryPid, startupCanTimeout);

    // Config steering motor PID
    steerMotor.config_kP(motionMagicPidSlot, 0.0, startupCanTimeout);
    steerMotor.config_kI(motionMagicPidSlot, 0.0, startupCanTimeout);
    steerMotor.config_kD(motionMagicPidSlot, 0.0, startupCanTimeout);
  }

  /**
   * Get the current state of the module.
   *
   * @return A SwerveModuleState representing the current speed and rotation of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        wheelMotor.getSelectedSensorVelocity() * ticksPer100msToMeterPerSec,
        Rotation2d.fromDegrees(steerMotor.getSelectedSensorPosition()));
  }

  /**
   * Set the current state of the swerve module.
   *
   * @param state The SwerveModuleState to set the module to
   */
  public void setState(SwerveModuleState state) {
    wheelMotor.set(
        TalonFXControlMode.Velocity, state.speedMetersPerSecond / ticksPer100msToMeterPerSec);
    steerMotor.set(
        TalonFXControlMode.MotionMagic, unwrapAngle(state.angle.getDegrees()) * degreesToTicks);
  }

  /**
   * Get the rotational speed of the modules.
   *
   * @return The modules steering speed in rads/s
   */
  @Log
  public double getSteeringSpeed() {
    return Math.toRadians(steeringEncoder.getVelocity());
  }

  @Log
  public double getSteeringAngle() { return steeringEncoder.getAbsolutePosition(); }

  public String configureLogName() {return moduleName;}

  /** Calibrate the absolute encoder. */
  public void calibrateAbsoluteEncoder() {
    steeringEncoder.configAllSettings(angleEncoderConfig);
    steeringEncoder.configMagnetOffset(0.0);
    steeringEncoder.configMagnetOffset(-steeringEncoder.getAbsolutePosition(), startupCanTimeout);
    steeringEncoder.setPositionToAbsolute(startupCanTimeout);
    steeringEncoder.configSetCustomParam(uniqueId, 0, startupCanTimeout);
    steeringEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    bootCanCoder();
  }

  private void bootCanCoder() {
    steeringEncoder.setStatusFramePeriod(
        CANCoderStatusFrame.SensorData, (int) (loopTime * 1000), startupCanTimeout);
    steeringEncoder.configAbsoluteSensorRange(
        AbsoluteSensorRange.Signed_PlusMinus180, startupCanTimeout);
    if (steeringEncoder.configGetCustomParam(0, startupCanTimeout) != uniqueId) {
      DriverStation.reportError("Swerve module[" + moduleName + "] needs to be recalibrate", false);
    }
  }

  private double unwrapAngle(double angle) {
    int turns = (int) (steerMotor.getSelectedSensorPosition() / 360.0);
    return angle + (turns * 360);
  }

  /** Stop the wheel and steering robot */
  public void stop() {
    steerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    wheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }
}
