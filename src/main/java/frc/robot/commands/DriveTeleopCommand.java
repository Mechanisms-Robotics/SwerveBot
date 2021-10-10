package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** Command to drive the swerve in teleop. Supplied left joystick x and y, and right joystick x. */
public class DriveTeleopCommand extends CommandBase {

  protected static final double MAX_TRANSLATIONAL_VELOCITY_RATE = 10; // m/s per second
  protected static final double MAX_ROTATION_VELOCITY_RATE = 4 * Math.PI; // rads/s per second

  protected static final double TRANSLATION_CURVE_STRENGTH = 10000.0;
  protected static final double ROTATION_CURVE_STRENGTH = 10.0; // 10.0 makes it effectively linear.

  protected static final double DEADBAND = 0.15;

  protected final Swerve swerve;

  protected final Supplier<Double> vxSupplier;
  protected final Supplier<Double> vySupplier;
  protected final Supplier<Double> vrxSupplier;
  protected final Supplier<Double> vrySupplier;
  protected final SlewRateLimiter vxRateLimiter;
  protected final SlewRateLimiter vyRateLimiter;
  protected final SlewRateLimiter vrRateLimiter;
  protected final boolean fieldOriented;
  protected final boolean forEnabled; // Field Oriented Rotation

  /**
   * Constructs the DriveTeleopCommand.
   *
   * @param driverX Left joystick x, which acts as desired x translation of the swerve drive
   * @param driverY Left joystick y, which acts as desired y translation of the swerve drive
   * @param driverRotationX The X component of the rotation vector.
   * @param driverRotationY The Y component of the rotation vector.
   * @param swerve Instance of Swerve
   */
  public DriveTeleopCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRotationX,
      Supplier<Double> driverRotationY,
      Swerve swerve) {
    vxSupplier = driverX;
    vySupplier = driverY;
    vrxSupplier = driverRotationX;
    vrySupplier = driverRotationY;

    this.fieldOriented = true;
    this.forEnabled = true;

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
    vrxSupplier = driverRotation;
    vrySupplier = null;

    this.fieldOriented = fieldOriented;
    this.forEnabled = false;

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
    double drx = vrxSupplier.get();
    dx = Math.abs(dx) > DEADBAND ? dx : 0;
    dy = Math.abs(dy) > DEADBAND ? dy : 0;
    drx = Math.abs(drx) > DEADBAND ? drx : 0;
    Translation2d translation = new Translation2d(dx, dy);
    double mag = translation.getNorm();
    final double scale = 1.35;
    mag = Math.pow(mag, scale);
    final Rotation2d rotation = new Rotation2d(translation.getX(), translation.getY());
    translation = new Translation2d(rotation.getCos() * mag, rotation.getSin() * mag);
    dx = vxRateLimiter.calculate(translation.getX() * Swerve.maxVelocity);
    dy = vyRateLimiter.calculate(translation.getY() * Swerve.maxVelocity);
    if (!forEnabled) drx = vrRateLimiter.calculate(drx * Swerve.maxRotationalVelocity);
    SmartDashboard.putNumber("Swerve vX", dx);
    SmartDashboard.putNumber("Swerve vY", dy);
    SmartDashboard.putNumber("Swerve vrX", drx);

    if (forEnabled) {
      double dry = vrySupplier.get();
      dry = Math.abs(dry) > DEADBAND ? dry : 0;
      SmartDashboard.putNumber("Swerve vrY", dry);
      Rotation2d rot = new Rotation2d(drx, dry);
      if (drx == 0 && dry == 0) rot = Rotation2d.fromDegrees(90.0);
      rot = rot.rotateBy(Rotation2d.fromDegrees(90));
      System.out.println(rot);
      swerve.drive(dx, dy, rot);
    } else {
      swerve.drive(dx, dy, drx, fieldOriented);
    }
  }

  private Rotation2d convertJoystickToAngle(double drx, double dry) {
    Translation2d joystickVec = new Translation2d(drx, dry);
    Translation2d forwardVec = new Translation2d(0.0, 1.0);
    double dotProduct =
        (joystickVec.getX() * forwardVec.getX()) + (joystickVec.getY() * forwardVec.getY());
    double magProduct = joystickVec.getNorm() * forwardVec.getNorm();
    double angle = Math.toDegrees(Math.acos(dotProduct / magProduct));
    return Rotation2d.fromDegrees(angle);
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
    return Math.signum(controlValue)
        * ((curveStrength * continuousControl) / (curveStrength - continuousControl + 1));
  }
}
