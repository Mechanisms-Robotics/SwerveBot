package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** Command to drive the swerve in teleop. Supplied left joystick x and y, and right joystick x. */
public class DriveTeleopFORCommand extends CommandBase {

  private static final double maxTranslationalVelocityRate = 4; // m/s per second
  private static final double maxRotationVelocityRate = 2 * Math.PI; // rads/s per second
  private static final double maxTranslationalVelocity = Swerve.maxVelocity * 0.90;
  private static final double maxRotationalVelocity = Swerve.maxRotationalVelocity;

  private static final double dxDeadband = 0.075;
  private static final double dyDeadband = 0.075;
  private static final double drDeadband = 0.075;

  private final Swerve swerve;

  private final Supplier<Double> vxSupplier;
  private final Supplier<Double> vySupplier;
  private final Supplier<Double> vrxSupplier;
  private final Supplier<Double> vrySupplier;
  private final Supplier<Boolean> forToggleButtonSupplier;

  private final SlewRateLimiter vxRateLimiter;
  private final SlewRateLimiter vyRateLimiter;
  private final SlewRateLimiter vrRateLimiter;

  private boolean forEnabled = false;
  private boolean prevFORToggled = false;
  private Rotation2d forHeading = new Rotation2d();

  /**
   * Constructs the DriveTeleopFORCommand.
   *
   * @param driverX Left joystick x, which acts as desired x translation of the swerve drive
   * @param driverY Left joystick y, which acts as desired y translation of the swerve drive
   * @param driverRotationX Right joystick x, which acts as the x value of the rotation vector
   * @param driverRotationY Right joystick y, which acts as the y value of the rotation vector
   * @param forToggleButton The button to toggle FOR (Field Oriented Rotation) mode.
   * @param swerve Instance of Swerve
   */
  public DriveTeleopFORCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRotationX,
      Supplier<Double> driverRotationY,
      Supplier<Boolean> forToggleButton,
      Swerve swerve) {
    vxSupplier = driverX;
    vySupplier = driverY;
    vrxSupplier = driverRotationX;
    vrySupplier = driverRotationY;
    forToggleButtonSupplier = forToggleButton;

    this.swerve = swerve;
    addRequirements(this.swerve);

    vxRateLimiter = new SlewRateLimiter(maxTranslationalVelocityRate);
    vyRateLimiter = new SlewRateLimiter(maxTranslationalVelocityRate);
    vrRateLimiter = new SlewRateLimiter(maxRotationVelocityRate);
  }

  @Override
  public void execute() {
    // Get driver input
    double dx = vxRateLimiter.calculate(vxSupplier.get() * maxTranslationalVelocity);
    double dy = vyRateLimiter.calculate(vySupplier.get() * maxTranslationalVelocity);
    double dr = vrRateLimiter.calculate(vrxSupplier.get() * maxRotationalVelocity);
    SmartDashboard.putNumber("Swerve vX", dx);
    SmartDashboard.putNumber("Swerve vY", dy);
    SmartDashboard.putNumber("Swerve vR", dr);

    dx = (Math.abs(dx) > this.dxDeadband) ? dx : 0;
    dy = (Math.abs(dy) > this.dyDeadband) ? dy : 0;
    dr = (Math.abs(dr) > this.drDeadband) ? dr : 0;

    if (forToggleButtonSupplier.get() && !prevFORToggled) {
      this.forEnabled = !this.forEnabled;
      swerve.setFOREnabled(this.forEnabled);
      prevFORToggled = true;
    } else if (!forToggleButtonSupplier.get() && prevFORToggled) {
      prevFORToggled = false;
    }

    if (forEnabled) {
      Translation2d rotationVec = new Translation2d(vrxSupplier.get(), vrySupplier.get());
      Translation2d forwardVec =
          new Translation2d(1.0, 0.0); // This will be the vector we get the angle from

      double angle = Math.atan2(rotationVec.getX(), forwardVec.getNorm()); // TODO: Angle unwrapping
      forHeading = Rotation2d.fromDegrees(angle);

      swerve.setFORDesiredHeading(forHeading);
      swerve.driveVelocity(dx, dy, 0, true);
    } else {
      swerve.driveVelocity(dx, dy, dr, true);
    }
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
