package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.SwerveKinematicController;

public class DriveTrajectoryCommand extends CommandBase {

    private

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDCommand thetaController;

    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final Supplier<Rotation2d> headingSupplier;
    private final Swerve swerve;
    private final SwerveKinematicController controller;

    private Rotation2d currentHeading;

    public DriveTrajectoryCommand(Trajectory trajectory, Swerve swerve, Supplier<Rotation2d> headingSupplier) {
        this.trajectory = trajectory;
        this.swerve = swerve;
        this.controller = swerve.getController();
        this.headingSupplier = headingSupplier;

        xController = new PIDController(controller.gainX, 0.0, 0.0);
        yController = new PIDController(controller.gainY, 0.0, 0.0);

        super.addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.reset();
        timer.start();
        controller.reset();
    }
    
    @Override
    public void execute() {
        if (headingSupplier != null) {
            currentHeading = headingSupplier.get();
        }
        final double currentTime = timer.get();

        final var desiredState = trajectory.sample(currentTime);

        final double xFF = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getCos();
        final double yFF = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getSin();
        final double rotationFF =

        final var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, rotationFF, swerve.getHeading());

        var moduleStates = controller.update(desiredState., currentStates, dt)

    }
}
