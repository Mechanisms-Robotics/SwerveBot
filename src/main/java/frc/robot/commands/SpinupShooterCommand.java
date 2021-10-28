package frc.robot.commands;

import static frc.robot.util.FlywheelSpeed.getFlywheelSpeed;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class SpinupShooterCommand extends CommandBase {
  public static final double TARGET_HEIGHT = 2.5; // Meters
  public static final double CAMERA_HEIGHT = 0.63881; // Meters
  public static final double CAMERA_PITCH = Units.degreesToRadians(60); // Radians

  public final double SPINDEXER_EXTRA_SPIN_UP_SPEED = 0.10;

  public final Shooter shooter;
  public final double spinupTime = 1.0;
  public final Timer timer = new Timer();
  public final PhotonCamera camera;
  private final NetworkTable photonNetworkTable;

  public SpinupShooterCommand(Shooter shooter, PhotonCamera camera) {
    this.shooter = shooter;
    this.camera = camera;
    this.photonNetworkTable =
            NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("limelight");

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  private double flywheelSpeed = Constants.shooterShootSpeed;
  @Override
  public void execute() {
    var cameraResults = camera.getLatestResult();
    if (cameraResults.hasTargets()) {
      double pitch = photonNetworkTable.getEntry("targetPitch").getDouble(0.0);
      var range =
              PhotonUtils.calculateDistanceToTargetMeters(
                      CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, Units.degreesToRadians(pitch));
      flywheelSpeed = getFlywheelSpeed(range);
    }
    shooter.setVelocity(flywheelSpeed);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(spinupTime);
  }
}
