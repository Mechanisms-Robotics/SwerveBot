package frc.robot.commands;

import static frc.robot.Constants.useSwerveVelocityControl;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.SwerveKinematicController;
import java.util.function.Supplier;

/** Command to drive the swerve in teleop. Supplied left joystick x and y, and right joystick x. */
public class DriveTeleopCommand extends CommandBase {

  private static final double maxTranslationalVelocityRate = 3.0; // m/s per second
  private static final double maxRotationVelocityRate = Math.PI; // rads/s per second
  private static final double maxTranslationalVelocity = Swerve.maxVelocity * 0.90;
  private static final double maxRotationalVelocity = Swerve.maxRotationalVelocity;

  private final Swerve swerve;
  private final SwerveKinematicController controller;
  private final Supplier<Double> vxSupplier;
  private final Supplier<Double> vySupplier;
  private final Supplier<Double> vrSupplier;
  private final SlewRateLimiter vxRateLimiter;
  private final SlewRateLimiter vyRateLimiter;
  private final SlewRateLimiter vrRateLimiter;

  private final boolean fieldOriented;
  private double lastLoop = -1;

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
    controller = swerve.getController();
    controller.reset();
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

    // Get the dt if we are using velocity control for the swerve
    if (useSwerveVelocityControl && lastLoop <= 0.0) {
      lastLoop = Timer.getFPGATimestamp();
      return;
    }
    final double dt = Timer.getFPGATimestamp() - lastLoop;

    // Get driver input
    final ChassisSpeeds desiredSpeed;
    final double dx = vxRateLimiter.calculate(vxSupplier.get() * maxTranslationalVelocity);
    final double dy = vyRateLimiter.calculate(vySupplier.get() * maxTranslationalVelocity);
    final double dr = vrRateLimiter.calculate(vrSupplier.get() * maxRotationalVelocity);

    // Correct for drive mode
    if (fieldOriented) {
      desiredSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(dx, dy, dr, swerve.getHeading());
    } else {
      desiredSpeed = new ChassisSpeeds(dx, dy, dr);
    }

    if (useSwerveVelocityControl) {
      double[][] speeds = controller.update(desiredSpeed, swerve.getModuleStates(), dt);
      swerve.setModuleSpeeds(speeds);
    } else {
      SwerveModuleState[] states = controller.getDesiredWheelStates(desiredSpeed);
      SwerveModuleState[] currentStates = swerve.getModuleStates();
      for (int i = 0; i < 4; i++) {
        states[i] = SwerveModuleState.optimize(states[i], currentStates[i].angle);
      }
      swerve.setModuleStates(states);
    }
    lastLoop = Timer.getFPGATimestamp();
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
