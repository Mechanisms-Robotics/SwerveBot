package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import org.photonvision.PhotonCamera;

import java.util.function.Supplier;

public class AimSwerveDrive extends DriveTeleopCommand {
  private static final PIDController aimPID = new PIDController(
          0.0,
          0.0,
          0.0,
          Constants.loopTime
  );

  private final PhotonCamera camera;

  public AimSwerveDrive(Supplier<Double> driverX,
                        Supplier<Double> driverY,
                        Supplier<Double> rotation,
                        Swerve swerve,
                        PhotonCamera camera) {
    super(driverX, driverY, rotation, swerve);
    this.camera = camera;
  }

  @Override
  public void execute() {

  }
}
