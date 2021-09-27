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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Spindexer extends SubsystemBase implements Loggable {
  private static final int SPINDEXER_MOTOR_ID = 30;
  private static final int RAMP_SOLENOID_FORWARD_ID = 2; // TODO: Get id from wiring
  private static final int RAMP_SOLENOID_REVERSE_ID = 3; // TODO: Get id from wiring
  private static final int GATE_SOLENOID_FORWARD_ID = 7; // TODO: Get id from wiring
  private static final int GATE_SOLENOID_REVERSE_ID = 0; // TODO: Get id from wiring

  private static final DoubleSolenoid.Value RAMP_DEPLOYED = DoubleSolenoid.Value.kForward;
  private static final DoubleSolenoid.Value RAMP_RETRACTED = DoubleSolenoid.Value.kReverse;
  private static final DoubleSolenoid.Value GATE_DEPLOYED = DoubleSolenoid.Value.kForward;
  private static final DoubleSolenoid.Value GATE_RETRACTED = DoubleSolenoid.Value.kReverse;

  private static final double SPINDEXER_GEAR_RATIO = 1.0;

  private static final TalonFXConfiguration SPINDEXER_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final int VELOCITY_PID_SLOT = 0;

  static {
    // TODO: Determine how much current the spindexer draws nominally and
    final var spindexerCurrentLimit = new SupplyCurrentLimitConfiguration();
    spindexerCurrentLimit.currentLimit = 10; // Amps
    spindexerCurrentLimit.triggerThresholdCurrent = 15; // Amps
    spindexerCurrentLimit.triggerThresholdTime = 0.5; // sec
    spindexerCurrentLimit.enable = true;
    SPINDEXER_MOTOR_CONFIG.supplyCurrLimit = spindexerCurrentLimit;

    // TODO: Tune
    final var velocityLoopConfig = new SlotConfiguration();
    velocityLoopConfig.kP = 0.0;
    velocityLoopConfig.kI = 0.0;
    velocityLoopConfig.kD = 0.0;
    SPINDEXER_MOTOR_CONFIG.slot0 = velocityLoopConfig;
  }

  private final WPI_TalonFX spindexerMotor = new WPI_TalonFX(SPINDEXER_MOTOR_ID);
  private final DoubleSolenoid rampSolenoid =
      new DoubleSolenoid(RAMP_SOLENOID_FORWARD_ID, RAMP_SOLENOID_REVERSE_ID);
  private final DoubleSolenoid gateSolenoid =
      new DoubleSolenoid(GATE_SOLENOID_FORWARD_ID, GATE_SOLENOID_REVERSE_ID);

  public Spindexer() {
    spindexerMotor.configAllSettings(SPINDEXER_MOTOR_CONFIG, startupCanTimeout);
    spindexerMotor.setInverted(TalonFXInvertType.Clockwise);
    spindexerMotor.setNeutralMode(NeutralMode.Coast);
    spindexerMotor.selectProfileSlot(VELOCITY_PID_SLOT, 0);

    deployGate();
  }

  public void setOpenLoop(double percentOutput) {
    spindexerMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setVelocity(double rpm) {
    spindexerMotor.set(ControlMode.Velocity, RPMToFalcon(rpm, SPINDEXER_GEAR_RATIO));
  }

  public void deployRamp() {
    rampSolenoid.set(RAMP_DEPLOYED);
  }

  public void retractRamp() {
    rampSolenoid.set(RAMP_RETRACTED);
  }

  public void deployGate() {
    gateSolenoid.set(GATE_DEPLOYED);
  }

  public void retractGate() {
    gateSolenoid.set(GATE_RETRACTED);
  }

  @Log
  public boolean isRampDeployed() {
    return rampSolenoid.get() == RAMP_DEPLOYED;
  }

  @Log
  public boolean isGateDeployed() {
    return gateSolenoid.get() == GATE_DEPLOYED;
  }

  @Log
  public double getVelocity() {
    return falconToRPM(spindexerMotor.getSelectedSensorVelocity(), SPINDEXER_GEAR_RATIO);
  }

  public void stop() {
    spindexerMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
