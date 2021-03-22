package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.FxSwerveModule;
import frc.robot.drivers.NavX;
import frc.robot.util.SwerveKinematicController;

/**
 * The base swerve drive class, controls all swerve modules in coordination.
 */
public class Swerve extends SubsystemBase {

  ///////////////
  // CONSTANTS //
  //////////////

  // The center of the robot is the origin point for all locations
  private static final double driveBaseWidth = 1.0; // m
  private static final double driveBaseLength = 1.0; // m

  private static final Translation2d flModuleLocation = new Translation2d(
      -driveBaseWidth / 2.0, driveBaseLength / 2.0
  );
  private static final Translation2d frModuleLocation = new Translation2d(
      driveBaseWidth / 2.0, driveBaseLength / 2.0
  );
  private static final Translation2d blModuleLocation = new Translation2d(
      -driveBaseWidth / 2.0, -driveBaseLength / 2.0
  );
  private static final Translation2d brModuleLocation = new Translation2d(
      driveBaseWidth / 2.0, -driveBaseLength / 2.0
  );

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

  private static final SwerveKinematicController kinematicController_ =
      new SwerveKinematicController(
          flModuleLocation,
          frModuleLocation,
          blModuleLocation,
          brModuleLocation
      );

  private static final double maxSpeed = 5; // m / s

  //////////////
  // HARDWARE //
  //////////////

  private final FxSwerveModule flModule = new FxSwerveModule(
      "Front Left", flWheelMotorID, flSteerMotorID, flSteerEncoderID
  );
  private final FxSwerveModule frModule = new FxSwerveModule(
      "Front Right", frWheelMotorID, frSteerMotorID, frSteerEncoderID
  );
  private final FxSwerveModule blModule = new FxSwerveModule(
      "Back Left", blWheelMotorID, blSteerMotorID, blSteerEncoderID
  );
  private final FxSwerveModule brModule = new FxSwerveModule(
      "Back Right", brWheelMotorID, brSteerMotorID, brSteerEncoderID
  );

  private final NavX gyro = new NavX(Port.kUSB);

  /////////////
  // METHODS //
  /////////////

  @Override
  public void periodic() {}

  public void drive(Translation2d desiredTranslation, double desiredRotation,
      boolean fieldRelative) {
    SwerveModuleState[] states = kinematicController_.getDesiredWheelStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            desiredTranslation.getX(),
            desiredTranslation.getY(),
            desiredRotation,
            getHeading())
            : new ChassisSpeeds(
                desiredTranslation.getX(),
                desiredTranslation.getY(),
                desiredRotation
            )
    );

    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.normalizeWheelSpeeds(states, maxSpeed);
    flModule.setState(states[0]);
    frModule.setState(states[1]);
    blModule.setState(states[2]);
    brModule.setState(states[3]);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public Rotation2d getHeading() {
    return gyro.getYaw();
  }

  public double getAngularVelocity() {
    return gyro.getYawRateRadiansPerSec();
  }
}
