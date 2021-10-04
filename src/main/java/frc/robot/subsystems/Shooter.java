package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.util.Units.RPMToFalcon;
import static frc.robot.util.Units.falconToRPM;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class Shooter extends SubsystemBase implements Loggable {
  private static final int SHOOTER_MOTOR_ID = 50;
  private static final int SHOOTER_FOLLOWER_MOTOR_ID = 51;
  private static final double SHOOTER_GEAR_RATIO = 1.09;

  private static final TalonFXConfiguration SHOOTER_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final int SHOOTER_PID_SLOT = 0;
  private static final SimpleMotorFeedforward FEEDFORWARD =
      new SimpleMotorFeedforward(
          0.0, // kS
          0.0, // kV
          0.0 // kA
          );

  static {
    // TODO: Determine how much current the shooter draws nominally and
    final var shooterCurrentLimit = new SupplyCurrentLimitConfiguration();
    shooterCurrentLimit.currentLimit = 30; // Amps
    shooterCurrentLimit.triggerThresholdCurrent = 40; // Amps
    shooterCurrentLimit.triggerThresholdTime = 0.2; // sec
    shooterCurrentLimit.enable = true;
    SHOOTER_MOTOR_CONFIG.supplyCurrLimit = shooterCurrentLimit;

    // TODO: Tune
    final var velocityLoopConfig = new SlotConfiguration();
    velocityLoopConfig.kP = 0.0;
    velocityLoopConfig.kI = 0.0;
    velocityLoopConfig.kD = 0.0;
    SHOOTER_MOTOR_CONFIG.slot0 = velocityLoopConfig;
  }

  private final WPI_TalonFX shooterMotor = new WPI_TalonFX(SHOOTER_MOTOR_ID);
  private final WPI_TalonFX shooterFollowMotor = new WPI_TalonFX(SHOOTER_FOLLOWER_MOTOR_ID);

  public Shooter() {
    shooterMotor.configAllSettings(SHOOTER_MOTOR_CONFIG, startupCanTimeout);
    shooterMotor.setInverted(TalonFXInvertType.Clockwise);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.selectProfileSlot(SHOOTER_PID_SLOT, 0);

    shooterFollowMotor.configAllSettings(SHOOTER_MOTOR_CONFIG, startupCanTimeout);
    shooterFollowMotor.follow(shooterMotor);
    shooterFollowMotor.setInverted(InvertType.OpposeMaster);
    shooterFollowMotor.setNeutralMode(NeutralMode.Coast);
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

  public double getVelocity() {
    return falconToRPM(shooterMotor.getSelectedSensorVelocity(), SHOOTER_GEAR_RATIO);
  }

  public void stop() {
    shooterMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
