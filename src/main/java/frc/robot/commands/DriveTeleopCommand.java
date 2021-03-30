package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** Command to drive the swerve in teleop. Supplied left joystick x and y, and right joystick x. */
public class DriveTeleopCommand extends CommandBase {
  private Swerve swerve;
  private Supplier<Double> translationX;
  private Supplier<Double> translationY;
  private Supplier<Double> rotation;

  private boolean fieldOriented;

  /**
   * Constructs the DriveTeleopCommand.
   *
   * @param translationX Left joystick x, which acts as desired x translation of the swerve drive
   * @param translationY Left joystick y, which acts as desired y translation of the swerve drive
   * @param rotation Right joystick x, which acts as desired rotation of the swerve drive
   * @param fieldOriented Whether or not driving is field oriented
   * @param swerve Instance of Swerve
   */
  public DriveTeleopCommand(
      Supplier<Double> translationX,
      Supplier<Double> translationY,
      Supplier<Double> rotation,
      boolean fieldOriented,
      Swerve swerve) {
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;

    this.fieldOriented = fieldOriented;

    this.swerve = swerve;
    addRequirements(this.swerve);
  }

  /**
   * Constructs the DriveTeleopCommand.
   *
   * @param translationX Left joystick x, which acts as desired x translation of the swerve drive
   * @param translationY Left joystick y, which acts as desired y translation of the swerve drive
   * @param rotation Right joystick x, which acts as desired rotation of the swerve drive
   * @param swerve Instance of Swerve
   */
  public DriveTeleopCommand(
      Supplier<Double> translationX,
      Supplier<Double> translationY,
      Supplier<Double> rotation,
      Swerve swerve) {
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;

    this.fieldOriented = false;

    this.swerve = swerve;
    addRequirements(this.swerve);
  }

  @Override
  public void execute() {
    Translation2d desiredTranslation = new Translation2d(translationX.get(), translationY.get());
    double desiredRotation = rotation.get();

    swerve.driveOpenLoop(desiredTranslation, desiredRotation, fieldOriented);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveOpenLoop(new Translation2d(), 0.0, false);
  }
}
