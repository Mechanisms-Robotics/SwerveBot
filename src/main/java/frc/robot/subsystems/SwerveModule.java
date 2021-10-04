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

  private static final double WHEEL_GEAR_RATIO = 6.86; // : 1
  private static final double WHEEL_DIAMETER = 0.1016; // meters
  private static final double STEER_GEAR_RATIO = 12.8; // : 1
  private static final int VELOCITY_PID_SLOT = 0;
  private static final int MOTION_MAGIC_PID_SLOT = 1;

  private static final TalonFXConfiguration WHEEL_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final TalonFXConfiguration STEERING_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final CANCoderConfiguration CONFIGURATION = new CANCoderConfiguration();

  private static final double STEERING_KP = 0.6;
  private static final double STEERING_KI = 0.0;
  private static final double STEERING_KD = 3.0;
  private static final double STEERING_DEADBAND = 75; // ticks

  private static final double WHEEL_KP = 0.01;
  private static final double WHEEL_KI = 0.0;
  private static final double WHEEL_KD = 0.0;
  private static final double WHEEL_KF = 0.05;
  private static final double WHEEL_KS = 0.0;
  private static final double WHEEL_KV = 0.0;
  private static final double WHEEL_KA = 0.0;
  private static final SimpleMotorFeedforward WHEEL_FEEDFORWARD =
      new SimpleMotorFeedforward(WHEEL_KS, WHEEL_KV, WHEEL_KA);

  static {
    // Wheel Motor Current Limiting
    var wheelMotorCurrentLimit = new SupplyCurrentLimitConfiguration();
    wheelMotorCurrentLimit.enable = true;
    wheelMotorCurrentLimit.currentLimit = 30; // Amps
    wheelMotorCurrentLimit.triggerThresholdCurrent = 35; // Amps
    wheelMotorCurrentLimit.triggerThresholdTime = 0.5; // Seconds
    WHEEL_MOTOR_CONFIG.supplyCurrLimit = wheelMotorCurrentLimit;

    // Configure wheel motor current limiting
    WHEEL_MOTOR_CONFIG.voltageCompSaturation = 12.0; // Volts

    // Configure steering motor current limiting
    var steeringMotorCurrentLimit = new SupplyCurrentLimitConfiguration();
    steeringMotorCurrentLimit.enable = true;
    steeringMotorCurrentLimit.currentLimit = 15; // Amps
    steeringMotorCurrentLimit.triggerThresholdCurrent = 25; // Amps
    steeringMotorCurrentLimit.triggerThresholdTime = 0.5; // Seconds
    STEERING_MOTOR_CONFIG.supplyCurrLimit = steeringMotorCurrentLimit;

    STEERING_MOTOR_CONFIG.voltageCompSaturation = 8.0; // Volts

    CONFIGURATION.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    CONFIGURATION.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    CONFIGURATION.sensorTimeBase = SensorTimeBase.PerSecond;
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
    wheelMotor.configAllSettings(WHEEL_MOTOR_CONFIG, startupCanTimeout);
    wheelMotor.setInverted(TalonFXInvertType.CounterClockwise);
    wheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, startupCanTimeout);

    // Configure wheel motor PID
    wheelMotor.config_kP(VELOCITY_PID_SLOT, WHEEL_KP, startupCanTimeout);
    wheelMotor.config_kI(VELOCITY_PID_SLOT, WHEEL_KI, startupCanTimeout);
    wheelMotor.config_kD(VELOCITY_PID_SLOT, WHEEL_KD, startupCanTimeout);
    wheelMotor.config_kF(VELOCITY_PID_SLOT, WHEEL_KF, startupCanTimeout);
    wheelMotor.setNeutralMode(NeutralMode.Brake);

    // Setup steering encoder
    steeringEncoder = new CANCoder(angleEncoderId);
    steeringEncoder.configAllSettings(CONFIGURATION);

    // Setup steering motor
    steerMotor = new WPI_TalonFX(steeringMotorId);
    steerMotor.configAllSettings(STEERING_MOTOR_CONFIG, startupCanTimeout);
    steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
    steerMotor.setNeutralMode(NeutralMode.Coast);
    steerMotor.selectProfileSlot(MOTION_MAGIC_PID_SLOT, 0);

    // Config steering motor PID
    steerMotor.config_kP(MOTION_MAGIC_PID_SLOT, STEERING_KP, startupCanTimeout);
    steerMotor.config_kI(MOTION_MAGIC_PID_SLOT, STEERING_KI, startupCanTimeout);
    steerMotor.config_kD(MOTION_MAGIC_PID_SLOT, STEERING_KD, startupCanTimeout);
    steerMotor.configAllowableClosedloopError(
        MOTION_MAGIC_PID_SLOT, STEERING_DEADBAND, startupCanTimeout);
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
    return falconToDegrees(steerMotor.getSelectedSensorPosition(), STEER_GEAR_RATIO);
  }

  /**
   * Get the wheel velocity in meters per second.
   *
   * @return Swerve wheel velocity in meters per second.
   */
  @Log
  public double getWheelVelocity() {
    return falconToMPS(
        wheelMotor.getSelectedSensorVelocity(), Math.PI * WHEEL_DIAMETER, WHEEL_GEAR_RATIO);
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
        MPSToFalcon(state.speedMetersPerSecond, Math.PI * WHEEL_DIAMETER, WHEEL_GEAR_RATIO);
    wheelMotor.set(
        ControlMode.Velocity,
        velocity,
        DemandType.ArbitraryFeedForward,
        WHEEL_FEEDFORWARD.calculate(velocity));

    // The speed of the wheel is really low (less than 0.01% Max speed) don't
    // steer. This prevents jittering
    final double angle =
        (Math.abs(state.speedMetersPerSecond) <= (Swerve.maxVelocity * 0.01))
            ? lastAngle
            : state.angle.getDegrees();
    steerMotor.set(ControlMode.Position, degreesToFalcon(angle, STEER_GEAR_RATIO));
    lastAngle = angle;
  }

  /** Configure the name of the log. Used by the Oblog logging system. */
  public String configureLogName() {
    return moduleName;
  }

  private void resetToAbsolute() {
    double absolutePosition = degreesToFalcon(getSteeringAngle().getDegrees(), STEER_GEAR_RATIO);
    steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  /** Stop the wheel and steering robot */
  public void stop() {
    steerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    wheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }
}
