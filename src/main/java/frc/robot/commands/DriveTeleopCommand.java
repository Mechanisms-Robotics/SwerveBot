package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** Command to drive the swerve in teleop. Supplied left joystick x and y, and right joystick x. */
public class DriveTeleopCommand extends CommandBase {

  private static final double MAX_TRANSLATIONAL_VELOCITY_RATE = 10; // m/s per second
  private static final double MAX_ROTATION_VELOCITY_RATE = 4 * Math.PI; // rads/s per second

  private static final double TRANSLATION_CURVE_STRENGTH = 0.4;
  private static final double ROTATION_CURVE_STRENGTH = 10.0; // 10.0 makes it effectively linear.

  private static final double DEADBAND = 0.2;

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

    vxRateLimiter = new SlewRateLimiter(MAX_TRANSLATIONAL_VELOCITY_RATE);
    vyRateLimiter = new SlewRateLimiter(MAX_TRANSLATIONAL_VELOCITY_RATE);
    vrRateLimiter = new SlewRateLimiter(MAX_ROTATION_VELOCITY_RATE);
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
    double dx = vxSupplier.get();
    double dy = vySupplier.get();
    double dr = vrSupplier.get();
    dx = applyControlCurve(Math.abs(dx) > DEADBAND ? dx : 0, TRANSLATION_CURVE_STRENGTH);
    dy = applyControlCurve(Math.abs(dy) > DEADBAND ? dy : 0, TRANSLATION_CURVE_STRENGTH);
    dr = applyControlCurve(Math.abs(dr) > DEADBAND ? dr : 0, ROTATION_CURVE_STRENGTH);
    dx = vxRateLimiter.calculate(dx * Swerve.maxVelocity);
    dy = vyRateLimiter.calculate(dy * Swerve.maxVelocity);
    dr = vrRateLimiter.calculate(dr * Swerve.maxRotationalVelocity);

    SmartDashboard.putNumber("Swerve vX", dx);
    SmartDashboard.putNumber("Swerve vY", dy);
    SmartDashboard.putNumber("Swerve vR", dr);



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

  /**
   * Applys a controll curve to the given input. This function does two things:
   *
   * <p>1. Remove the effect of the deadband.
   *
   * <p>2. Map the control signal onto a normalized signum function.
   *
   * @param controlValue The control signal to map.
   * @param curveStrength The strength of the signum function. Note that values between -1.0 and 0.0
   *     will lead to bad results. Go to See <a
   *     href="https://www.desmos.com/calculator/zzpkh9cujz">Desmos</a> to see how curveStrength (k)
   *     affects the output.
   * @return The mapped control value.
   */
  private double applyControlCurve(double controlValue, double curveStrength) {
    // Get rid of the discontinuity that is caused by setting any value between -DEADBAND and
    // DEADBAND to 0.
    final double absControl = Math.abs(controlValue);
    final double continuousControl = (absControl - DEADBAND) * (1.0 / (1.0 - DEADBAND));
    return Math.signum(continuousControl)
        * ((curveStrength * continuousControl) / (curveStrength - continuousControl + 1));
  }
}
