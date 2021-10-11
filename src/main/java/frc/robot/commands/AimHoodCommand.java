package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  public static final double TARGET_HEIGHT = 2.5; // Meters
  public static final double CAMERA_HEIGHT = 0.63881; // Meters
  public static final double CAMERA_PITCH = Units.degreesToRadians(60); // Radians

  private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      RANGE_TO_HOOD_MAP = new InterpolatingTreeMap<>();

  static {
    RANGE_TO_HOOD_MAP.put(new InterpolatingDouble(0.2), new InterpolatingDouble(-1.0));
    RANGE_TO_HOOD_MAP.put(new InterpolatingDouble(1.1), new InterpolatingDouble(-1.0));
    RANGE_TO_HOOD_MAP.put(new InterpolatingDouble(1.4), new InterpolatingDouble(-0.609));
    RANGE_TO_HOOD_MAP.put(new InterpolatingDouble(1.63), new InterpolatingDouble(-0.4));
    RANGE_TO_HOOD_MAP.put(new InterpolatingDouble(1.896), new InterpolatingDouble(-0.16));
}

  private Hood hood;
  private PhotonCamera camera;
  private double targetHoodPosition = 0.0;

  private final NetworkTable photonNetworkTable;

  public AimHoodCommand(PhotonCamera camera, Hood hood) {
    this.hood = hood;
    this.camera = camera;

    this.photonNetworkTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("limelight");

    addRequirements(hood);
  }

  @Override
  public void execute() {
    var cameraResults = camera.getLatestResult();
    if (cameraResults.hasTargets()) {
      double pitch = photonNetworkTable.getEntry("targetPitch").getDouble(0.0);
      var range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT,
              TARGET_HEIGHT,
              CAMERA_PITCH,
              Units.degreesToRadians(pitch));
      targetHoodPosition = RANGE_TO_HOOD_MAP.getInterpolated(new InterpolatingDouble(range)).value;
    }
    hood.setHoodRawPosition(targetHoodPosition);
  }
}
