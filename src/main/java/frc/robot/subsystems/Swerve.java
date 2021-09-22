package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** The base swerve drive class, controls all swerve modules in coordination. */
public class Swerve extends SubsystemBase implements Loggable {

  public static final double maxVelocity = 4.5; // m / s
  public static final double maxRotationalVelocity = Math.PI; // rads/s

  // The center of the robot is the origin point for all locations
  private static final double driveBaseWidth = 0.44; // m
  private static final double driveBaseLength = 0.44; // m

  private static final Translation2d flModuleLocation =
      new Translation2d(driveBaseLength / 2.0, driveBaseWidth / 2.0);
  private static final Translation2d frModuleLocation =
      new Translation2d(driveBaseLength / 2.0, -driveBaseWidth / 2.0);
  private static final Translation2d blModuleLocation =
      new Translation2d(-driveBaseLength / 2.0, driveBaseWidth / 2.0);
  private static final Translation2d brModuleLocation =
      new Translation2d(-driveBaseLength / 2.0, -driveBaseWidth / 2.0);

  private static final int flWheelMotorID = 12;
  private static final int flSteerMotorID = 13;
  private static final int flSteerEncoderID = 12;
  private static final int frWheelMotorID = 14;
  private static final int frSteerMotorID = 15;
  private static final int frSteerEncoderID = 14;
  private static final int blWheelMotorID = 10;
  private static final int blSteerMotorID = 11;
  private static final int blSteerEncoderID = 10;
  private static final int brWheelMotorID = 16;
  private static final int brSteerMotorID = 17;
  private static final int brSteerEncoderID = 16;

  private static final double flAngleOffset = 11.250;
  private static final double frAngleOffset = 52.646;
  private static final double blAngleOffset = 204.697;
  private static final double brAngleOffset = 221.045;

  private static final int gyroID = 1;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          flModuleLocation, frModuleLocation, blModuleLocation, brModuleLocation);

  private final SwerveDrivePoseEstimator poseEstimator;

  private final SwerveModule flModule =
      new SwerveModule(
          "Front Left", flWheelMotorID, flSteerMotorID, flSteerEncoderID, flAngleOffset);
  private final SwerveModule frModule =
      new SwerveModule(
          "Front Right", frWheelMotorID, frSteerMotorID, frSteerEncoderID, frAngleOffset);
  private final SwerveModule blModule =
      new SwerveModule(
          "Back Left", blWheelMotorID, blSteerMotorID, blSteerEncoderID, blAngleOffset);
  private final SwerveModule brModule =
      new SwerveModule(
          "Back Right", brWheelMotorID, brSteerMotorID, brSteerEncoderID, brAngleOffset);

  private final PigeonIMU gyro = new PigeonIMU(gyroID);

  private PIDController headingStabilizationPID;
  private Rotation2d stabilizationHeading = new Rotation2d();
  private boolean turning;

  // Field Oriented Rotation
  private boolean forEnabled = false;
  private PIDController forPID;
  private Rotation2d forHeading = new Rotation2d();

  /** Constructs the Swerve subsystem. */
  public Swerve() {
    poseEstimator =
        new SwerveDrivePoseEstimator(
            getHeading(),
            new Pose2d(),
            kinematics,
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(Units.degreesToRadians(0.01)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    headingStabilizationPID = new PIDController(0.005, 0.0, 0.0);
    headingStabilizationPID.setTolerance(1.0);

    // TODO: Tune
    forPID = new PIDController(0.005, 0.0, 0.0);
    forPID.setTolerance(5.0);

    this.register();
    this.setName("Swerve Drive");
  }

  @Override
  public void periodic() {
    poseEstimator.update(
        getHeading(),
        flModule.getState(),
        frModule.getState(),
        blModule.getState(),
        brModule.getState());
  }

  public void driveVelocity(
      double xVelocity, double yVelocity, double rotationVelocity, boolean fieldRelative) {

    // Heading stabilization loop
    if (Math.abs(rotationVelocity) < 0.01) {
      // If we where turning then stoped turning lock in current heading
      if (turning) {
        stabilizationHeading = getHeading();
        turning = false;
      } else {
        if (forEnabled)
          rotationVelocity += forPID.calculate(getHeading().getDegrees(), forHeading.getDegrees());

        rotationVelocity +=
            headingStabilizationPID.calculate(
                getHeading().getDegrees(), stabilizationHeading.getDegrees());
      }
    } else {
      turning = true;
    }

    // If field relative is updated
    ChassisSpeeds speeds;
    if (fieldRelative) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xVelocity, yVelocity, rotationVelocity, getHeading());
    } else {
      speeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.normalizeWheelSpeeds(states, maxVelocity);
    setModuleStates(states);
  }

  /**
   * Sets all the swerve module states sequentially.
   *
   * @param states What to set the states to
   */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.normalizeWheelSpeeds(states, maxVelocity);
    flModule.setState(states[0]);
    frModule.setState(states[1]);
    blModule.setState(states[2]);
    brModule.setState(states[3]);
  }

  /**
   * Get the states of all the current modules.
   *
   * @return A list of SwerveModulesState(s)
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = flModule.getState();
    states[1] = frModule.getState();
    states[2] = blModule.getState();
    states[3] = brModule.getState();
    return states;
  }

  /** Sets whether to use FOR (Field Oriented Rotation) control or not. * */
  public void setFOREnabled(boolean isEnabled) {
    this.forEnabled = isEnabled;
  }

  /** Updates desired heading for FOR (Field Oriented Rotation) control. * */
  public void setFORDesiredHeading(Rotation2d desiredHeading) {
    forHeading = desiredHeading;
  }

  /** Zeros the gyro heading. */
  public void zeroHeading() {
    gyro.setYaw(0.0);
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns the swerve drive's heading as a Rotation2d.
   *
   * @return A Rotation2d representing the swerve drive's heading
   */
  @Log(methodName = "getDegrees", name = "Gyro Heading")
  public Rotation2d getHeading() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return Rotation2d.fromDegrees(ypr[0]);
  }

  /**
   * Get the current pose of the robot.
   *
   * @return A Pose2d that represents the position of the robot
   */
  @Log(methodName = "getX", name = "Drive Estimated X")
  @Log(methodName = "getY", name = "Drive Estimated Y")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Stop all motors on the drive train */
  public void stop() {
    flModule.stop();
    frModule.stop();
    blModule.stop();
    brModule.stop();
  }
}
