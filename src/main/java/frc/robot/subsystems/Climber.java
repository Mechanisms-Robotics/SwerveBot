package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

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

import org.ejml.equation.Variable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Climber extends SubsystemBase implements Loggable {
    private static final int CLIMBER_MOTOR_ID = 50;
    private static final double CLIMBER_GEAR_RATIO = (1/16);

    private static final TalonFXConfiguration CLIMBER_MOTOR_CONFIG = new TalonFXConfiguration();

    static {
        final var climberCurrentLimit = new SupplyCurrentLimitConfiguration();
        climberCurrentLimit.currentLimit = 10; // Amps
        climberCurrentLimit.triggerThresholdCurrent = 15; // Amps
        climberCurrentLimit.triggerThresholdTime = 0.5; // sec
        climberCurrentLimit.enable = true;
        CLIMBER_MOTOR_CONFIG.supplyCurrLimit = climberCurrentLimit;


        //Change Slot # for Motor Config
        final var velocityLoopConfig = new SlotConfiguration();
        CLIMBER_MOTOR_CONFIG.slot0 = velocityLoopConfig;
    }
    private final WPI_TalonFX climberMotor = new WPI_TalonFX(CLIMBER_MOTOR_ID);

    public Climber() {

        //TODO: Double
        climberMotor.configAllSettings(CLIMBER_MOTOR_CONFIG, startupCanTimeout);
        climberMotor.setInverted(TalonFXInvertType.Clockwise);
        climberMotor.setNeutralMode(NeutralMode.Brake);
    }
    public void setOpenLoop(double percentOutput) {
        climberMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    @Log
    public double getVelocity() {
        return falconToRPM(climberMotor.getSelectedSensorVelocity(), CLIMBER_GEAR_RATIO);
    }

    public void stop() {
        climberMotor.set(ControlMode.PercentOutput, 0.0);
    }
}