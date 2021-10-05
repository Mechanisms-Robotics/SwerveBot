package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** Command to drive the swerve in teleop. Supplied left joystick x and y, and right joystick x. */
public class DriveTeleopCommand extends CommandBase {

  private static final double maxTranslationalVelocityRate = 4; // m/s per second
  private static final double maxRotationVelocityRate = 2 * Math.PI; // rads/s per second
  private static final double maxTranslationalVelocity = Swerve.maxVelocity;
  private static final double maxRotationalVelocity = Swerve.maxRotationalVelocity;

  private static final double dxDeadband = 0.075; // Joystick percentage
  private static final double dyDeadband = 0.075; // Joystick percentage
  private static final double drDeadband = 0.075; // Joystick percentage

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
    // Get driver input
    double dx = vxRateLimiter.calculate(vxSupplier.get() * maxTranslationalVelocity);
    double dy = vyRateLimiter.calculate(vySupplier.get() * maxTranslationalVelocity);
    double dr = vrRateLimiter.calculate(vrSupplier.get() * maxRotationalVelocity);
    SmartDashboard.putNumber("Swerve vX", dx);
    SmartDashboard.putNumber("Swerve vY", dy);
    SmartDashboard.putNumber("Swerve vR", dr);

    dx = (Math.abs(dx) > dxDeadband) ? dx : 0;
    dy = (Math.abs(dy) > dyDeadband) ? dy : 0;
    dr = (Math.abs(dr) > drDeadband) ? dr : 0;

    swerve.drive(dx, dy, dr, fieldOriented);
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
