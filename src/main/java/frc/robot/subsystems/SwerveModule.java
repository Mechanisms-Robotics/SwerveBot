// Code based on Team 3847
// https://github.com/Spectrum3847/GammaRay-2021/blob/main/src/main/java/frc/robot/subsystems/SwerveModule.java

package frc.robot.subsystems;

import static frc.robot.Constants.falconCPR;
import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.Constants.talonPrimaryPid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.util.CTREModuleState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** A hardware wrapper class for a swerve module that uses Falcon500s. */
public class SwerveModule implements Loggable {

  private static final double moduleGearRatio = 6.86; // : 1
  private static final double wheelDiameter = 0.1016; // meters
  private static final double ticksPer100msToMeterPerSec =
      ((wheelDiameter * Math.PI) / (falconCPR * moduleGearRatio)) * 10.0;
  private static final double steerGearRatio = 12.8; // : 1
  private static final double degreesToTicks = (falconCPR * steerGearRatio) / 360;
  private static final int velocityPidSlot = 0;
  private static final int motionMagicPidSlot = 1;

  private static final TalonFXConfiguration wheelMotorConfig = new TalonFXConfiguration();
  private static final TalonFXConfiguration steeringMotorConfig = new TalonFXConfiguration();
  private static final CANCoderConfiguration angleEncoderConfig = new CANCoderConfiguration();

  private static final double steeringKp = 0.4;
  private static final double steeringKi = 0.0;
  private static final double steeringKd = 3.0;

  private static final double wheelKp = 0.10;
  private static final double wheelKi = 0.0;
  private static final double wheelKd = 0.0;
  private static final double wheelKs = 0.0;
  private static final double wheelKv = 0.0;
  private static final double wheelKa = 0.0;
  private static final SimpleMotorFeedforward wheelFeedforward =
      new SimpleMotorFeedforward(wheelKs, wheelKv, wheelKa);

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

    angleEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    angleEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    angleEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }

  private final WPI_TalonFX wheelMotor;
  private final WPI_TalonFX steerMotor;
  private final CANCoder steeringEncoder;
  private final String moduleName;

  @Log.ToString private SwerveModuleState desiredState;

  @Log private double lastAngle;
  private double angleOffset;

  /**
   * Constructs a swerve module.
   *
   * @param name The unique name of this swerve module
   * @param wheelMotorId The CAN ID of the Spark Max used to control the wheel
   * @param steeringMotorId The CAN ID of the Spark Max used to control the steering
   * @param angleEncoderId The CAN ID of the CANCoder used to determine the angle of the module
   */
  public SwerveModule(
      String name, int wheelMotorId, int steeringMotorId, int angleEncoderId, double angleOffset) {

    moduleName = name;
    this.angleOffset = angleOffset;

    // Setup wheel motor
    wheelMotor = new WPI_TalonFX(wheelMotorId);
    wheelMotor.configAllSettings(wheelMotorConfig, startupCanTimeout);
    wheelMotor.setInverted(TalonFXInvertType.CounterClockwise);
    wheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, startupCanTimeout);

    // Configure wheel motor PID
    wheelMotor.config_kP(velocityPidSlot, wheelKp, startupCanTimeout);
    wheelMotor.config_kI(velocityPidSlot, wheelKi, startupCanTimeout);
    wheelMotor.config_kD(velocityPidSlot, wheelKd, startupCanTimeout);
    wheelMotor.setNeutralMode(NeutralMode.Brake);

    // Setup steering encoder
    steeringEncoder = new CANCoder(angleEncoderId);
    steeringEncoder.configAllSettings(angleEncoderConfig);

    // Setup steering motor
    steerMotor = new WPI_TalonFX(steeringMotorId);
    steerMotor.configAllSettings(steeringMotorConfig, startupCanTimeout);
    steerMotor.setInverted(TalonFXInvertType.Clockwise);
    steerMotor.setSelectedSensorPosition(
        steeringEncoder.getAbsolutePosition() * degreesToTicks, talonPrimaryPid, startupCanTimeout);
    steerMotor.setNeutralMode(NeutralMode.Coast);

    // Config steering motor PID
    steerMotor.config_kP(motionMagicPidSlot, steeringKp, startupCanTimeout);
    steerMotor.config_kI(motionMagicPidSlot, steeringKi, startupCanTimeout);
    steerMotor.config_kD(motionMagicPidSlot, steeringKd, startupCanTimeout);
    resetToAbsolute();
  }

  @Log.ToString
  public Rotation2d getSteeringAngle() {
    return Rotation2d.fromDegrees(steeringEncoder.getAbsolutePosition());
  }
  /**
   * Get the current state of the module.
   *
   * @return A SwerveModuleState representing the current speed and rotation of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        wheelMotor.getSelectedSensorVelocity() * ticksPer100msToMeterPerSec, getSteeringAngle());
  }

  /**
   * Set the current state of the swerve module.
   *
   * @param state The SwerveModuleState to set the module to
   */
  public void setState(SwerveModuleState state) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    final double velocity = desiredState.speedMetersPerSecond / ticksPer100msToMeterPerSec;
    wheelMotor.set(
        ControlMode.Velocity,
        velocity,
        DemandType.ArbitraryFeedForward,
        wheelFeedforward.calculate(velocity));

    // The speed of the wheel is really low (less than 0.01% Max speed) don't
    // steer. This prevents jittering
    final double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.maxVelocity * 0.01))
            ? lastAngle
            : desiredState.angle.getDegrees();
    steerMotor.set(ControlMode.Position, angle / degreesToTicks);
    lastAngle = angle;
  }

  public String configureLogName() {
    return moduleName;
  }

  private void resetToAbsolute() {
    double absolutePosition = (getSteeringAngle().getDegrees() - angleOffset) * degreesToTicks;
    steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  /** Stop the wheel and steering robot */
  public void stop() {
    steerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    wheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }
}
