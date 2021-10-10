package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Climber;
import java.util.function.Supplier;

public class ClimberCommand extends CommandBase {
  private final Supplier<Double> climberJoystick;

  private final Climber climber;

  private static final double CLIMBER_MAX_SPEED = 0.25; // percent

  public ClimberCommand(Supplier<Double> climberJoystick, Climber climber) {
    this.climberJoystick = climberJoystick;

    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    double climberSpeed = climberJoystick.get();
    MathUtil.clamp(climberSpeed, -CLIMBER_MAX_SPEED, CLIMBER_MAX_SPEED);
    climber.setOpenLoop(climberSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
