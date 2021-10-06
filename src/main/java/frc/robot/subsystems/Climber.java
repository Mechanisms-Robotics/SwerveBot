package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.util.Units.falconToRPM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class Climber extends SubsystemBase implements Loggable {
  private static final int CLIMBER_MOTOR_ID = 60;
  private static final double CLIMBER_GEAR_RATIO = 16.0;

  private static final TalonFXConfiguration CLIMBER_MOTOR_CONFIG = new TalonFXConfiguration();

  static {
    // TODO: Determine how much current the climber draws nominally and
    final var climberCurrentLimit = new SupplyCurrentLimitConfiguration();
    climberCurrentLimit.currentLimit = 10; // Amps
    climberCurrentLimit.triggerThresholdCurrent = 15; // Amps
    climberCurrentLimit.triggerThresholdTime = 0.5; // sec
    climberCurrentLimit.enable = true;
    CLIMBER_MOTOR_CONFIG.supplyCurrLimit = climberCurrentLimit;

    CLIMBER_MOTOR_CONFIG.reverseSoftLimitEnable = true;
    CLIMBER_MOTOR_CONFIG.reverseSoftLimitThreshold = 0;
  }

  private final WPI_TalonFX climberMotor = new WPI_TalonFX(CLIMBER_MOTOR_ID);

  public Climber() {
    // TODO: Double
    climberMotor.configAllSettings(CLIMBER_MOTOR_CONFIG, startupCanTimeout);
    climberMotor.setInverted(TalonFXInvertType.Clockwise);
    climberMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setOpenLoop(double percentOutput) {
    climberMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public double getVelocity() {
    return falconToRPM(climberMotor.getSelectedSensorVelocity(), CLIMBER_GEAR_RATIO);
  }

  public void stop() {
    climberMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
