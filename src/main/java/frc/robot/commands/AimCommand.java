package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class AimCommand extends ParallelCommandGroup {

  public AimCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> rotation,
      Hood hood,
      Swerve swerve,
      Shooter shooter,
      Accelerator accelerator,
      Spindexer spindexer,
      PhotonCamera camera) {
    addRequirements(hood, swerve, shooter, accelerator, spindexer);
    addCommands(
        new InstantCommand(() -> camera.setLED(VisionLEDMode.kOn)),
        new SpinupCommand(shooter, accelerator, spindexer),
        new AimHoodCommand(camera, hood),
        new AimSwerveDrive(driverX, driverY, rotation, swerve, camera));
  }
}
