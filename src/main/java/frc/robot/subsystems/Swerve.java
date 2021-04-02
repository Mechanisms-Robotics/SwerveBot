package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.util.SwerveKinematicController;

/** The base swerve drive class, controls all swerve modules in coordination. */
public class Swerve extends SubsystemBase {

  public static final double maxVelocity = 4.5; // m / s
  public static final double maxRotationalVelocity = Math.PI; // rads/s

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

  private final SwerveKinematicController kinematicController =
      new SwerveKinematicController(
          flModuleLocation, frModuleLocation, blModuleLocation, brModuleLocation);

  private final SwerveDrivePoseEstimator poseEstimator;

  private final FxSwerveModule flModule =
      new FxSwerveModule("Front Left", flWheelMotorID, flSteerMotorID, flSteerEncoderID);
  private final FxSwerveModule frModule =
      new FxSwerveModule("Front Right", frWheelMotorID, frSteerMotorID, frSteerEncoderID);
  private final FxSwerveModule blModule =
      new FxSwerveModule("Back Left", blWheelMotorID, blSteerMotorID, blSteerEncoderID);
  private final FxSwerveModule brModule =
      new FxSwerveModule("Back Right", brWheelMotorID, brSteerMotorID, brSteerEncoderID);

  private final PigeonIMU gyro = new PigeonIMU(0);

  private Pose2d currentPose = new Pose2d();
  private boolean velocityControl = false;
  private ChassisSpeeds desiredSpeed = new ChassisSpeeds();
  private Pose2d desiredPose = null;
  private double lastUpdate = -1;

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

    // Update the pose of the robot
    currentPose =
        poseEstimator.update(
            getHeading(),
            flModule.getState(),
            frModule.getState(),
            blModule.getState(),
            brModule.getState());

    // If we don't have a previous update then wait for the
    // next update so we can calculate the correct loop time
    if (lastUpdate < 0) {
      lastUpdate = Timer.getFPGATimestamp();
      return;
    }

    double dt = Timer.getFPGATimestamp() - lastUpdate;
    SwerveModuleState[] currentStates = getModuleStates();

    // Update the swerve module control
    if (velocityControl) {
      double[][] speeds;
      if (desiredPose == null) {
        speeds = kinematicController.update(desiredSpeed, currentStates, dt);
      } else {
        speeds =
            kinematicController.update(currentPose, desiredPose, desiredSpeed, currentStates, dt);
      }
      setModuleSpeeds(speeds);
    } else {
      SwerveModuleState[] states;
      if (desiredPose == null) {
        states = kinematicController.getDesiredWheelStates(desiredSpeed);
      } else {
        states = kinematicController.getDesiredWheelStates(desiredSpeed);
      }
      for (int i = 0; i < 4; i++) {
        states[i] = SwerveModuleState.optimize(states[i], currentStates[i].angle);
      }
      setModuleStates(states);
    }
    lastUpdate = Timer.getFPGATimestamp();
  }

  /**
   * Controls the swerve modules in coordination to drive a desired translation and rotation.
   *
   * @param dx Speed in m/s in the x direction
   * @param dy Speed in m/s in the y direction
   * @param dr Rotation speed in rads/s
   * @param fieldRelative True if the speeds are felid relative otherwise false
   */
  public void driveOpenLoop(double dx, double dy, double dr, boolean fieldRelative) {
    if (desiredPose != null) {
      this.desiredPose = null;
      kinematicController.reset();
    }

    if (fieldRelative) {
      desiredSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(dx, dy, dr, getHeading());
    } else {
      desiredSpeed = new ChassisSpeeds(dx, dy, dr);
    }
  }

  /**
   * Controls the swerve modules in coordination to drive to a desired point and be at a desired
   * speed at that point. Note that all values MUST be field relative unlike the open loop method.
   *
   * @param desiredSpeed The desired speed to have at the desired pose
   * @param desiredPose The desired pose to be at
   */
  public void driveClosedLoop(ChassisSpeeds desiredSpeed, Pose2d desiredPose) {
    if (this.desiredPose == null) {
      kinematicController.reset();
    }
    this.desiredPose = desiredPose;
    this.desiredSpeed =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            desiredSpeed.vxMetersPerSecond,
            desiredSpeed.vyMetersPerSecond,
            desiredSpeed.omegaRadiansPerSecond,
            getHeading());
  }

  /**
   * Reset the kinematic controller. This should be called every time the robot changes to a
   * different trajectory\control state.
   */
  public void resetController() {
    kinematicController.reset();
  }

  /**
   * Weather or not to use all velocity control on the swerve module.
   *
   * @param useVelocityMode True for velocity mode false for non-velocity mode
   */
  public void setVelocityMode(boolean useVelocityMode) {
    velocityControl = useVelocityMode;
  }

  /**
   * Sets all of the swerve module states sequentially.
   *
   * @param states What to set the states to
   */
  private void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.normalizeWheelSpeeds(states, maxVelocity);
    flModule.setState(states[0]);
    frModule.setState(states[1]);
    blModule.setState(states[2]);
    brModule.setState(states[3]);
  }

  private void setModuleSpeeds(double[][] speeds) {
    flModule.setVelocity(speeds[0][0], speeds[0][1]);
    frModule.setVelocity(speeds[1][0], speeds[1][1]);
    blModule.setVelocity(speeds[2][0], speeds[2][1]);
    brModule.setVelocity(speeds[3][0], speeds[3][1]);
  }

  /**
   * Get the states of all the current modules.
   *
   * @return A list of SwerevModulesState(s)
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = flModule.getState();
    states[1] = frModule.getState();
    states[2] = blModule.getState();
    states[3] = brModule.getState();
    return states;
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
