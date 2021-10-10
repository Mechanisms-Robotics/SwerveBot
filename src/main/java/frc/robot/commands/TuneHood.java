package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

public class TuneHood extends CommandBase {

  private final Hood hood;
  private final PhotonCamera camera;

  public TuneHood(Hood hood, PhotonCamera camera) {
    this.camera = camera;
    this.hood = hood;
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
      var range =
          PhotonUtils.calculateDistanceToTargetMeters(
              AimHoodCommand.CAMERA_HEIGHT,
              AimHoodCommand.TARGET_HEIGHT,
              AimHoodCommand.CAMERA_PITCH,
              Units.degreesToRadians(cameraResults.getBestTarget().getPitch()));
      SmartDashboard.putNumber("Range", range);
      SmartDashboard.putNumber("Hood Servo Position", hood.getCurrentPos());
    }
  }

  @Override
  public void end(boolean interrupted) {
    camera.setLED(VisionLEDMode.kOff);
    camera.setDriverMode(true);
  }
}
