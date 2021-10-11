package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import frc.robot.subsystems.Hood;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

public class TuneHood extends CommandBase{

  private final Hood hood;
  private final PhotonCamera camera;

  private final NetworkTable photonNetworkTable;

  public TuneHood(Hood hood, PhotonCamera camera) {
    this.camera = camera;
    this.hood = hood;

    this.photonNetworkTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("limelight");
  }

  @Override
  public void initialize() {
    camera.setLED(VisionLEDMode.kOn);
    camera.setPipelineIndex(0);
    camera.setDriverMode(false);
  }

  @Override
  public void execute() {
    var cameraResults = camera.getLatestResult();
    SmartDashboard.putBoolean("Has Target", cameraResults.hasTargets());
    if (cameraResults.hasTargets()) {
      double pitch = photonNetworkTable.getEntry("targetPitch").getDouble(0.0);
      SmartDashboard.putNumber("Camera Pitch", pitch);
      var range =
          PhotonUtils.calculateDistanceToTargetMeters(
              AimHoodCommand.CAMERA_HEIGHT,
              AimHoodCommand.TARGET_HEIGHT,
              AimHoodCommand.CAMERA_PITCH,
              Units.degreesToRadians(pitch));
      SmartDashboard.putNumber("Range", range);
    }
    SmartDashboard.putNumber("Hood Servo Position", hood.getCurrentPos());
  }

  @Override
  public void end(boolean interrupted) {
    camera.setLED(VisionLEDMode.kOff);
    camera.setDriverMode(true);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
