package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
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
import frc.robot.drivers.FxSwerveModule;
import frc.robot.util.SwerveKinematicController;

/** The base swerve drive class, controls all swerve modules in coordination. */
public class Swerve extends SubsystemBase {

  // The center of the robot is the origin point for all locations
  private static final double driveBaseWidth = 1.0; // m
  private static final double driveBaseLength = 1.0; // m

  private static final Translation2d flModuleLocation =
      new Translation2d(-driveBaseWidth / 2.0, driveBaseLength / 2.0);
  private static final Translation2d frModuleLocation =
      new Translation2d(driveBaseWidth / 2.0, driveBaseLength / 2.0);
  private static final Translation2d blModuleLocation =
      new Translation2d(-driveBaseWidth / 2.0, -driveBaseLength / 2.0);
  private static final Translation2d brModuleLocation =
      new Translation2d(driveBaseWidth / 2.0, -driveBaseLength / 2.0);

  private static final int flWheelMotorID = 0;
  private static final int flSteerMotorID = 1;
  private static final int flSteerEncoderID = 0;
  private static final int frWheelMotorID = 2;
  private static final int frSteerMotorID = 3;
  private static final int frSteerEncoderID = 1;
  private static final int blWheelMotorID = 4;
  private static final int blSteerMotorID = 5;
  private static final int blSteerEncoderID = 2;
  private static final int brWheelMotorID = 6;
  private static final int brSteerMotorID = 7;
  private static final int brSteerEncoderID = 3;

  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          flModuleLocation, frModuleLocation, blModuleLocation, brModuleLocation);

  private static final SwerveKinematicController kinematicController =
      new SwerveKinematicController(
          flModuleLocation, frModuleLocation, blModuleLocation, brModuleLocation);

  private static SwerveDrivePoseEstimator poseEstimator;
  private static Pose2d currentPose = new Pose2d();

  private static final double maxSpeed = 4.5; // m / s

  private final FxSwerveModule flModule =
      new FxSwerveModule("Front Left", flWheelMotorID, flSteerMotorID, flSteerEncoderID);
  private final FxSwerveModule frModule =
      new FxSwerveModule("Front Right", frWheelMotorID, frSteerMotorID, frSteerEncoderID);
  private final FxSwerveModule blModule =
      new FxSwerveModule("Back Left", blWheelMotorID, blSteerMotorID, blSteerEncoderID);
  private final FxSwerveModule brModule =
      new FxSwerveModule("Back Right", brWheelMotorID, brSteerMotorID, brSteerEncoderID);

  private final PigeonIMU gyro = new PigeonIMU(0);

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
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  /** Updates the odometry based using the SwerveDrivePoseEstimator. */
  private void updateOdometry() {
    currentPose =
        poseEstimator.update(
            getHeading(),
            flModule.getState(),
            frModule.getState(),
            blModule.getState(),
            brModule.getState());
  }

  /**
   * Controls the swerve modules in coordination to drive a desired translation and rotation.
   *
   * @param desiredTranslation The desired translation to drive
   * @param desiredRotation The desired rotation to drive
   * @param fieldRelative Whether or not to drive realtive to the field
   */
  public void driveOpenLoop(
      Translation2d desiredTranslation, double desiredRotation, boolean fieldRelative) {
    SwerveModuleState[] states =
        kinematicController.getDesiredWheelStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    desiredTranslation.getX(),
                    desiredTranslation.getY(),
                    desiredRotation,
                    getHeading())
                : new ChassisSpeeds(
                    desiredTranslation.getX(), desiredTranslation.getY(), desiredRotation));

    setModuleStates(states);
  }

  /**
   * Sets all of the swerve module states sequentially.
   *
   * @param states What to set the states to
   */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.normalizeWheelSpeeds(states, maxSpeed);
    flModule.setState(states[0]);
    frModule.setState(states[1]);
    blModule.setState(states[2]);
    brModule.setState(states[3]);
  }

  /** Zeros the gryo heading. */
  public void zeroHeading() {
    gyro.setYaw(0.0);
  }

  /** Calibrates all the swerve module's CANCoders. (Only run this when the modules are zeroed) */
  public void calibrateSwerveModules() {
    flModule.calibrateAbsoluteEncoder();
    frModule.calibrateAbsoluteEncoder();
    blModule.calibrateAbsoluteEncoder();
    brModule.calibrateAbsoluteEncoder();
  }

  /**
   * Returns the swerve drive's heading as a Rotation2d.
   *
   * @return A Rotation2d representing the swerve drive's heading
   */
  public Rotation2d getHeading() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return Rotation2d.fromDegrees(ypr[0]);
  }
}
