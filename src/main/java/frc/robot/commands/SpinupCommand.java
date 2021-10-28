package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import org.photonvision.PhotonCamera;

public class SpinupCommand extends ParallelCommandGroup {

    public SpinupCommand(Spindexer spindexer, Accelerator accelerator, Shooter shooter, PhotonCamera camera) {
        addCommands(
                new SpinupSpindexerCommand(accelerator, spindexer),
                new SpinupShooterCommand(shooter, camera)
        );
    }
}
