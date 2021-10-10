package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

public class AimHoodCommand extends CommandBase {
  private static final int PIPELINE_INDEX = 0;

  // TODO: Measure
  public static final double TARGET_HEIGHT = 2; // Meters
  public static final double CAMERA_HEIGHT = 0.5; // Meters
  public static final double CAMERA_PITCH = 0.0; // Radians

  private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      RANGE_TO_HOOD_MAP = new InterpolatingTreeMap<>();

  static {
    // TODO Add hood range points here
  }

  private Hood hood;
  private PhotonCamera camera;
  private double targetHoodPosition = 0.0;

  public AimHoodCommand(PhotonCamera camera, Hood hood) {
    this.hood = hood;
    this.camera = camera;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    var cameraResults = camera.getLatestResult();
    if (cameraResults.hasTargets()) {
      var range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT,
              TARGET_HEIGHT,
              CAMERA_PITCH,
              Units.degreesToRadians(cameraResults.getBestTarget().getPitch()));
      targetHoodPosition = RANGE_TO_HOOD_MAP.get(range).value;
    }
    hood.setHoodRawPosition(targetHoodPosition);
  }
}
