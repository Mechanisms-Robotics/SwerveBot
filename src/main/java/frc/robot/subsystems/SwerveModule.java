// Code based on Team 3847
// https://github.com/Spectrum3847/GammaRay-2021/blob/main/src/main/java/frc/robot/subsystems/SwerveModule.java

package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.util.Units.*;

import com.ctre.phoenix.motorcontrol.*;
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

  private static final double wheelGearRatio = 6.86; // : 1
  private static final double wheelDiameter = 0.1016; // meters
  private static final double steerGearRatio = 12.8; // : 1
  private static final int velocityPidSlot = 0;
  private static final int motionMagicPidSlot = 1;

  private static final TalonFXConfiguration wheelMotorConfig = new TalonFXConfiguration();
  private static final TalonFXConfiguration steeringMotorConfig = new TalonFXConfiguration();
  private static final CANCoderConfiguration angleEncoderConfig = new CANCoderConfiguration();

  private static final double steeringKp = 0.2;
  private static final double steeringKi = 0.0;
  private static final double steeringKd = 3.0;

  private static final double wheelKp = 0.01;
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

  private double lastAngle;
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
    steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
    steerMotor.setNeutralMode(NeutralMode.Coast);
    steerMotor.selectProfileSlot(motionMagicPidSlot, 0);

    // Config steering motor PID
    steerMotor.config_kP(motionMagicPidSlot, steeringKp, startupCanTimeout);
    steerMotor.config_kI(motionMagicPidSlot, steeringKi, startupCanTimeout);
    steerMotor.config_kD(motionMagicPidSlot, steeringKd, startupCanTimeout);
    resetToAbsolute();
  }

  /**
   * Get the steering angle based on the absolute encoder
   *
   * @return Steering angle in degrees.
   */
  @Log.ToString
  public Rotation2d getSteeringAngle() {
    return Rotation2d.fromDegrees(steeringEncoder.getAbsolutePosition() - angleOffset);
  }

  /**
   * Get the steering angle based on the internal falcon encoder.
   *
   * @return Steering angle in degrees.
   */
  @Log
  public double getSteeringAngleMotor() {
    return falconToDegrees(steerMotor.getSelectedSensorPosition(), steerGearRatio);
  }

  /**
   * Get the wheel velocity in meters per second.
   *
   * @return Swerve wheel velocity in meters per second.
   */
  @Log
  public double getWheelVelocity() {
    return falconToMPS(
        wheelMotor.getSelectedSensorVelocity(), Math.PI * wheelDiameter, wheelGearRatio);
  }

  /**
   * Get the current state of the module.
   *
   * @return A SwerveModuleState representing the current speed and rotation of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getWheelVelocity(), Rotation2d.fromDegrees(getSteeringAngleMotor()));
  }

  /**
   * Set the current state of the swerve module.
   *
   * @param state The SwerveModuleState to set the module to
   */
  public void setState(SwerveModuleState state) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // CTRE is not
    state = CTREModuleState.optimize(state, getState().angle);
    final double velocity =
        MPSToFalcon(state.speedMetersPerSecond, Math.PI * wheelDiameter, wheelGearRatio);
    wheelMotor.set(
        ControlMode.Velocity,
        velocity,
        DemandType.ArbitraryFeedForward,
        wheelFeedforward.calculate(velocity));

    // The speed of the wheel is really low (less than 0.01% Max speed) don't
    // steer. This prevents jittering
    final double angle =
        (Math.abs(state.speedMetersPerSecond) <= (Swerve.maxVelocity * 0.01))
            ? lastAngle
            : state.angle.getDegrees();
    steerMotor.set(ControlMode.Position, degreesToFalcon(angle, steerGearRatio));
    lastAngle = angle;
  }

  /** Configure the name of the log. Used by the Oblog logging system. */
  public String configureLogName() {
    return moduleName;
  }

  private void resetToAbsolute() {
    double absolutePosition = degreesToFalcon(getSteeringAngle().getDegrees(), steerGearRatio);
    steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  /** Stop the wheel and steering robot */
  public void stop() {
    steerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    wheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }
}
