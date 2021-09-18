package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.util.Units.RPMToFalcon;
import static frc.robot.util.Units.falconToRPM;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {
  private static final int SHOOTER_MOTOR_ID = 40;
  private static final int SHOOTER_FOLLOWER_MOTOR_ID = 41;
  private static final double SHOOTER_GEAR_RATIO = 1.0;

  private static final TalonFXConfiguration SHOOTER_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final int SHOOTER_PID_SLOT = 0;
  private static final SimpleMotorFeedforward FEEDFORWARD =
      new SimpleMotorFeedforward(
          0.0, // kS
          0.0, // kV
          0.0 // kA
          );

  private static final int HOOD_SERVO_PWM_PORT = 0;

  static {
    // TODO: Determine how much current the spindexer draws nominally and
    final var acceleratorCurrentLimit = new SupplyCurrentLimitConfiguration();
    acceleratorCurrentLimit.currentLimit = 10; // Amps
    acceleratorCurrentLimit.triggerThresholdCurrent = 15; // Amps
    acceleratorCurrentLimit.triggerThresholdTime = 0.5; // sec
    acceleratorCurrentLimit.enable = true;
    SHOOTER_MOTOR_CONFIG.supplyCurrLimit = acceleratorCurrentLimit;

    // TODO: Tune
    final var velocityLoopConfig = new SlotConfiguration();
    velocityLoopConfig.kP = 0.0;
    velocityLoopConfig.kI = 0.0;
    velocityLoopConfig.kD = 0.0;
    SHOOTER_MOTOR_CONFIG.slot0 = velocityLoopConfig;
  }

  private final WPI_TalonFX shooterMotor = new WPI_TalonFX(SHOOTER_MOTOR_ID);
  private final WPI_TalonFX shooterFollowMotor = new WPI_TalonFX(SHOOTER_FOLLOWER_MOTOR_ID);
  private final Servo hoodServo = new Servo(HOOD_SERVO_PWM_PORT);

  public Shooter() {
    shooterMotor.configAllSettings(SHOOTER_MOTOR_CONFIG, startupCanTimeout);
    shooterMotor.setInverted(TalonFXInvertType.CounterClockwise);
    shooterMotor.setNeutralMode(NeutralMode.Brake);
    shooterMotor.selectProfileSlot(SHOOTER_PID_SLOT, 0);

    shooterFollowMotor.configAllSettings(SHOOTER_MOTOR_CONFIG, startupCanTimeout);
    shooterFollowMotor.follow(shooterMotor);
    shooterFollowMotor.setInverted(InvertType.FollowMaster);
    shooterMotor.setNeutralMode(NeutralMode.Brake);

    hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    ;
  }

  public void setOpenLoop(double percentOutput) {
    shooterMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setVelocity(double rpm) {
    shooterMotor.set(
        ControlMode.Velocity,
        RPMToFalcon(rpm, SHOOTER_GEAR_RATIO),
        DemandType.ArbitraryFeedForward,
        FEEDFORWARD.calculate(rpm));
  }

  @Log
  public double getVelocity() {
    return falconToRPM(shooterMotor.getSelectedSensorVelocity(), SHOOTER_GEAR_RATIO);
  }

  public void stop() {
    shooterMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setHoodRawPosition(double rawPosition) {
    MathUtil.clamp(rawPosition, -1.0, 1.0);

    // Set speed is the speed of the pwm pulse not the speed of the servo
    // PWM speed commands the servo position.
    hoodServo.setSpeed(rawPosition);
  }
}
