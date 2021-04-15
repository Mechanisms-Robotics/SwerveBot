package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/** High-level kinematic controller for a swerve robot. */
public class SwerveKinematicController {

  // Gains for the kinematic controller
  public final double gainX; // meters per sec per meter per sec (of error)
  public final double gainY; // meters per sec per meter per sec (of error)
  public final double gainTheta;
  private final double gainSteering;

  private final Pose2d[] moduleLocations;
  private Rotation2d[] lastControl;
  private boolean reset;

  /**
   * Full constructor for the kinematic controller.
   *
   * @param gainX How responsive the controls is to positional error in the X (forward/back)
   *     direction.
   * @param gainY How responsive the controls is to positional error in the Y (left/right)
   *     direction.
   * @param gainTheta How responsive the controls is to positional error in the X (forward/back)
   *     direction.
   * @param gainSteering How aggressive the controller is when steering modules to there correct
   *     steering position. (Higher gain means faster steering)
   * @param moduleLocations The locations of the modules on the swerve drive.
   */
  public SwerveKinematicController(
      double gainX,
      double gainY,
      double gainTheta,
      double gainSteering,
      Translation2d... moduleLocations) {
    this.gainX = gainX;
    this.gainY = gainY;
    this.gainTheta = gainTheta;
    this.gainSteering = gainSteering;

    this.moduleLocations = new Pose2d[moduleLocations.length];
    for (int i = 0; i < moduleLocations.length; i++) {
      this.moduleLocations[i] = new Pose2d(moduleLocations[i], new Rotation2d());
    }
    lastControl = new Rotation2d[moduleLocations.length];
    reset = true;
  }

  /**
   * Constructor that uses the default gains of the controller.
   *
   * @param moduleLocations The locations of the modules on the swerve drive.
   */
  public SwerveKinematicController(Translation2d... moduleLocations) {
    this(1.0, 1.0, 0.5, 1.0, moduleLocations);
  }

  /**
   * Update the kinematic controller. This needs to be called every iteration of the swerve control
   * loop.
   *
   * <p>This update function is intended for use in trajectory tracking<\p>
   *
   * @param pose The current pose of the robot.
   * @param reference The desired robot pose.
   * @param desiredRobotSpeed The desired robot speed at the reference pose.
   * @param currentStates The current state of each module. The length of the array is is deterged
   *     by the number of moduleLocations passed to the constructor of this object.
   * @param dt The time between the last loop iteration and now.
   * @return An array that contains the wheel velocity and the steering speeds of each module. The
   *     array is in the same order as the order that the moduleLocations where passed to the
   *     constructor. Each subareas is formatted as [wheelVelocity, steeringVelocity]. Steering
   *     velocity's are counter-clockwise postie.
   */
  public double[][] update(
      Pose2d pose,
      Pose2d reference,
      ChassisSpeeds desiredRobotSpeed,
      SwerveModuleState[] currentStates,
      double dt) {
    SwerveModuleState[] controlStates =
        getControlWheelStates(pose, reference, currentStates, desiredRobotSpeed);

    for (int i = 0; i < moduleLocations.length; i++) {
      controlStates[i] = SwerveModuleState.optimize(controlStates[i], currentStates[i].angle);
    }

    double[][] controlSpeeds = new double[moduleLocations.length][2];
    double[] steeringSpeeds = getControlSteerSpeeds(controlStates, currentStates, dt);
    for (int i = 0; i < moduleLocations.length; i++) {
      controlSpeeds[i][0] = controlStates[i].speedMetersPerSecond;
      controlSpeeds[i][1] = steeringSpeeds[i];
    }
    return controlSpeeds;
  }

  /**
   * Update the kinematic controller. This needs to be called every iteration of the swerve control
   * loop.
   *
   * <p>This update function is intended for open-loop control (e.g. driver control)<\p>
   *
   * @param desiredRobotSpeed The speed the robot should be at.
   * @param currentStates The current state of each swerve module. The length of the array is is
   *     deterged by the number of moduleLocations passed to the contractor of this object.
   * @param dt The time between the last loop iteration and now.
   * @return An array that contains the wheel velocity and the steering speeds of each module. The
   *     array is in the same order as the order that the moduleLocations where passed to the
   *     constructor. Each subareas is formatted as [wheelVelocity, steeringVelocity]. Steering
   *     velocity's are counter-clockwise postie.
   */
  public double[][] update(
      ChassisSpeeds desiredRobotSpeed, SwerveModuleState[] currentStates, double dt) {
    SwerveModuleState[] controlStates = getDesiredWheelStates(desiredRobotSpeed);

    for (int i = 0; i < moduleLocations.length; i++) {
      controlStates[i] = SwerveModuleState.optimize(controlStates[i], currentStates[i].angle);
    }

    double[][] controlSpeeds = new double[moduleLocations.length][2];
    double[] steeringSpeeds = getControlSteerSpeeds(controlStates, currentStates, dt);
    for (int i = 0; i < moduleLocations.length; i++) {
      controlSpeeds[i][0] = controlStates[i].speedMetersPerSecond;
      controlSpeeds[i][1] = steeringSpeeds[i];
    }
    return controlSpeeds;
  }

  /**
   * Reset the kinematic controller.
   *
   * <p>This should be called every time that the swerve drive swithers to a new control state.
   * (e.g. when robot is enabled\switching to a new trajectory)
   */
  public void reset() {
    reset = true;
  }

  /**
   * Get the end-states of the swerve drive modules given a desired robot velocity.
   *
   * @param desiredRobotSpeed The desired robot velocity.
   * @return A list of swerve module states in order of that each module location was passed to the
   *     contractor.
   */
  public SwerveModuleState[] getDesiredWheelStates(ChassisSpeeds desiredRobotSpeed) {
    SwerveModuleState[] states = new SwerveModuleState[moduleLocations.length];
    for (int i = 0; i < moduleLocations.length; i++) {
      states[i] = getDesiredWheelState(desiredRobotSpeed, moduleLocations[i]);
    }
    return states;
  }

  /**
   * Get the intermediate control states of the swerve drive given a desired position and velocity.
   *
   * @param pose The current pose of the robot.
   * @param reference The desired pose of the robot.
   * @param currentStates The current states of the swerve modules.
   * @param desiredRobotSpeed The desired robot speed at the reference pose.
   * @return A list of swerve module states in order of that each module location was passed to the
   *     contractor.
   */
  public SwerveModuleState[] getControlWheelStates(
      Pose2d pose,
      Pose2d reference,
      SwerveModuleState[] currentStates,
      ChassisSpeeds desiredRobotSpeed) {
    SwerveModuleState[] controlStates = new SwerveModuleState[moduleLocations.length];
    Pose2d error = pose.relativeTo(reference);
    for (int i = 0; i < moduleLocations.length; i++) {
      controlStates[i] =
          getControlWheelState(
              pose, error, desiredRobotSpeed, moduleLocations[i], currentStates[i].angle);
    }
    return controlStates;
  }

  private SwerveModuleState getControlWheelState(
      Pose2d pose,
      Pose2d error,
      ChassisSpeeds desiredRobotSpeed,
      Pose2d wheelLocation,
      Rotation2d currentSteeringAngle) {
    Pose2d wheelGlobalLocation = wheelLocation.relativeTo(pose);
    SwerveModuleState noErrorState = getDesiredWheelState(desiredRobotSpeed, wheelLocation);

    double z;
    double z1;
    double z2;
    z = noErrorState.angle.minus(currentSteeringAngle).getRadians();
    if (Math.abs(z) <= Math.PI / 2) {
      z1 = 1;
      z2 = 0;
    } else {
      z1 = -1;
      z2 = Math.PI;
    }

    double a =
        noErrorState.speedMetersPerSecond * noErrorState.angle.getCos()
            + gainX * error.getX()
            - gainTheta * wheelGlobalLocation.getY() * error.getRotation().getRadians();
    double b =
        noErrorState.speedMetersPerSecond * noErrorState.angle.getSin()
            + gainY * error.getY()
            - gainTheta * wheelGlobalLocation.getX() * error.getRotation().getRadians();

    return new SwerveModuleState(z1 * Math.hypot(a, b), new Rotation2d(z2 + Math.atan2(b, a)));
  }

  private double[] getControlSteerSpeeds(
      SwerveModuleState[] controlStates, SwerveModuleState[] currentStates, double dt) {
    double[] speeds = new double[moduleLocations.length];
    for (int i = 0; i < speeds.length; i++) {

      // This makes sure that we don't get discontinuities in the velocity of the control
      // state.
      if (reset) {
        reset = false;
        speeds[i] =
            controlStates[i].angle.minus(currentStates[i].angle).getRadians() * gainSteering;
      } else {
        speeds[i] =
            controlStates[i].angle.minus(lastControl[i]).getRadians() / dt
                + controlStates[i].angle.minus(currentStates[i].angle).getRadians() * gainSteering;
      }
      lastControl[i] = controlStates[i].angle;
    }
    return speeds;
  }

  private static SwerveModuleState getDesiredWheelState(
      ChassisSpeeds desiredRobotSpeed, Pose2d wheelLocation) {
    double vxWheel =
        desiredRobotSpeed.vxMetersPerSecond
            - wheelLocation.getY() * desiredRobotSpeed.omegaRadiansPerSecond;
    double vyWheel =
        desiredRobotSpeed.vyMetersPerSecond
            + wheelLocation.getX() * desiredRobotSpeed.omegaRadiansPerSecond;
    return new SwerveModuleState(Math.hypot(vxWheel, vyWheel), new Rotation2d(vyWheel, vxWheel));
  }
}
