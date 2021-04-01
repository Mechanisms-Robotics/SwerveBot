package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** Command to drive the swerve in teleop. Supplied left joystick x and y, and right joystick x. */
public class DriveTeleopCommand extends CommandBase {

  private static final double maxTranslationalVelocityRate = 3.0; // m/s per second
  private static final double maxRotationVelocityRate = Math.PI; // rads/s per second
  private static final double maxTranslationalVelocity = Swerve.maxVelocity * 0.90;
  private static final double maxRotationalVelocity = Swerve.maxRotationalVelocity;

  private final Swerve swerve;
  private final Supplier<Double> vxSupplier;
  private final Supplier<Double> vySupplier;
  private final Supplier<Double> vrSupplier;
  private final SlewRateLimiter vxRateLimiter;
  private final SlewRateLimiter vyRateLimiter;
  private final SlewRateLimiter vrRateLimiter;

  private final boolean fieldOriented;

  /**
   * Constructs the DriveTeleopCommand.
   *
   * @param driverX Left joystick x, which acts as desired x translation of the swerve drive
   * @param driverY Left joystick y, which acts as desired y translation of the swerve drive
   * @param driverRotation Right joystick x, which acts as desired rotation of the swerve drive
   * @param fieldOriented Whether or not driving is field oriented
   * @param swerve Instance of Swerve
   */
  public DriveTeleopCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRotation,
      boolean fieldOriented,
      Swerve swerve) {
    vxSupplier = driverX;
    vySupplier = driverY;
    vrSupplier = driverRotation;

    this.fieldOriented = fieldOriented;

    this.swerve = swerve;
    this.swerve.resetController();
    addRequirements(this.swerve);

    vxRateLimiter = new SlewRateLimiter(maxTranslationalVelocityRate);
    vyRateLimiter = new SlewRateLimiter(maxTranslationalVelocityRate);
    vrRateLimiter = new SlewRateLimiter(maxRotationVelocityRate);
  }

  /**
   * Constructs the DriveTeleopCommand.
   *
   * @param driverX Left joystick x, which acts as desired x translation of the swerve drive
   * @param driverY Left joystick y, which acts as desired y translation of the swerve drive
   * @param driverRotation Right joystick x, which acts as desired rotation of the swerve drive
   * @param swerve Instance of Swerve
   */
  public DriveTeleopCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRotation,
      Swerve swerve) {
    this(driverX, driverY, driverRotation, true, swerve);
  }

  @Override
  public void execute() {
    swerve.driveOpenLoop(
        vxRateLimiter.calculate(vxSupplier.get() * maxTranslationalVelocity),
        vyRateLimiter.calculate(vySupplier.get() * maxTranslationalVelocity),
        vrRateLimiter.calculate(vrSupplier.get() * maxRotationalVelocity),
        fieldOriented);
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveOpenLoop(0.0, 0.0, 0.0, fieldOriented);
  }
}
