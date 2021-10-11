package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

public class AimSwerveDrive extends DriveTeleopCommand {
  private static final PIDController aimPID = new PIDController(0.0, 0.0, 0.0, Constants.loopTime);

  private final PhotonCamera camera;
  private final NetworkTable photonNetworkTable;

  public AimSwerveDrive(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> rotation,
      Swerve swerve,
      PhotonCamera camera) {
    super(driverX, driverY, rotation, swerve);
    this.camera = camera;

    // TODO: Figure out bug with usenig camera results for these values
    photonNetworkTable =
        NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("limelight");
  }

  @Override
  public void execute() {
    var cameraResults = camera.getLatestResult();
    if (cameraResults.hasTargets()) {
      final double yaw = photonNetworkTable.getEntry("targetYaw").getDouble(0.0);
      final double pidOutput = aimPID.calculate(yaw);
      super.driveRotationVelocityMode(
          deadband(vxSupplier.get()), deadband(vySupplier.get()), pidOutput);
    } else {
      super.execute();
    }
  }
}
