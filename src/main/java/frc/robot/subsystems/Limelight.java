package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This acts as a wrapper class for our Limelight. It handles targeting so our shooter can aim at
 * the power port. *
 */
public class Limelight extends SubsystemBase {
  private final NetworkTable llNetworkTable =
      NetworkTableInstance.getDefault().getTable("limelight");

  private static class LLRawData {
    /* Inputs */
    public int currentLEDMode;
    public int currentPipeline;
    public double xOffset;
    public double yOffset;
    public boolean hasTarget;

    /* Outputs */
    public int wantedLEDMode = 1;
    public int wantedPipeline = 0;
  }

  public enum LedMode {
    PIPELINE, OFF, BLINK, ON
  }

  private final LLRawData rawData = new LLRawData();

  public Limelight() {}

  @Override
  public void periodic() {
    /* Read periodic inputs */
    rawData.currentLEDMode = (int) llNetworkTable.getEntry("ledMode").getDouble(1.0);
    rawData.currentPipeline = (int) llNetworkTable.getEntry("pipeline").getDouble(0.0);
    rawData.xOffset = llNetworkTable.getEntry("tx").getDouble(0.0);
    rawData.yOffset = llNetworkTable.getEntry("ty").getDouble(0.0);
    rawData.hasTarget = (llNetworkTable.getEntry("tv").getDouble(0.0) == 1.0);

    /* Write periodic outputs */
    if (rawData.currentLEDMode != rawData.wantedLEDMode
        || rawData.currentPipeline != rawData.wantedPipeline) {
      llNetworkTable.getEntry("ledMode").setNumber(rawData.wantedLEDMode);
      llNetworkTable.getEntry("pipeline").setNumber(rawData.wantedPipeline);
    }
  }

  public void setLEDMode(LedMode ledMode) {
    rawData.wantedLEDMode = ledMode.ordinal();
  }

  public void setLEDMode(int ledMode) {
    rawData.wantedLEDMode = ledMode;
  }

  public void setPipeline(int pipeline) {
    rawData.wantedPipeline = pipeline;
  }
}
