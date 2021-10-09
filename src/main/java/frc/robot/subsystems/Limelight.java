package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * This acts as a wrapper class for our Limelight. It handles targeting so our shooter can aim at
 * the power port. *
 */
public class Limelight extends SubsystemBase {

  private PhotonCamera camera = new PhotonCamera("limelight");
  private PhotonPipelineResult result = new PhotonPipelineResult();

  public Limelight() {}

  @Override
  public void periodic() {
    /* Read periodic inputs */
    result = camera.getLatestResult();
  }

  public PhotonPipelineResult getResult() {
    return result;
  }

  public void setLEDMode(VisionLEDMode mode) {
    camera.setLED(mode);
  }

  public void setPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }
}
