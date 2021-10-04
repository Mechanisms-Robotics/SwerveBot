package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.util.Units.RPMToFalcon;
import static frc.robot.util.Units.falconToRPM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Accelerator extends SubsystemBase implements Loggable {
  private static final int ACCELERATOR_MOTOR_ID = 40;

  private static final double ACCELERATOR_GEAR_RATIO = 2.25;

  private static final TalonFXConfiguration ACCELERATOR_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final int VELOCITY_PID_SLOT = 0;

  static {
    // TODO: Determine how much current the spindexer draws nominally and
    final var acceleratorCurrentLimit = new SupplyCurrentLimitConfiguration();
    acceleratorCurrentLimit.currentLimit = 10; // Amps
    acceleratorCurrentLimit.triggerThresholdCurrent = 15; // Amps
    acceleratorCurrentLimit.triggerThresholdTime = 0.5; // sec
    acceleratorCurrentLimit.enable = true;
    ACCELERATOR_MOTOR_CONFIG.supplyCurrLimit = acceleratorCurrentLimit;

    // TODO: Tune
    final var velocityLoopConfig = new SlotConfiguration();
    velocityLoopConfig.kP = 0.0;
    velocityLoopConfig.kI = 0.0;
    velocityLoopConfig.kD = 0.0;
    ACCELERATOR_MOTOR_CONFIG.slot0 = velocityLoopConfig;
  }

  private final WPI_TalonFX acceleratorMotor = new WPI_TalonFX(ACCELERATOR_MOTOR_ID);

  public Accelerator() {
    acceleratorMotor.configAllSettings(ACCELERATOR_MOTOR_CONFIG, startupCanTimeout);
    acceleratorMotor.setInverted(TalonFXInvertType.CounterClockwise);
    acceleratorMotor.setNeutralMode(NeutralMode.Coast);
    acceleratorMotor.selectProfileSlot(VELOCITY_PID_SLOT, 0);
  }

  public void setOpenLoop(double percentOutput) {
    acceleratorMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setVelocity(double rpm) {
    acceleratorMotor.set(ControlMode.Velocity, RPMToFalcon(rpm, ACCELERATOR_GEAR_RATIO));
  }

  /**
   * Get the velocity of the accelerator rollers.
   *
   * @return Velocity in RPM
   */
  @Log
  public double getVelocity() {
    return falconToRPM(acceleratorMotor.getSelectedSensorVelocity(), ACCELERATOR_GEAR_RATIO);
  }

  public void stop() {
    acceleratorMotor.setNeutralMode(NeutralMode.Brake);
    acceleratorMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void coast() {
    acceleratorMotor.setNeutralMode(NeutralMode.Coast);
    acceleratorMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
