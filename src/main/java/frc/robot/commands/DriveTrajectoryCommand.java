package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.SwerveKinematicController;

public class DriveTrajectoryCommand extends CommandBase {

    private final Trajectory trajectory;
    private final Swerve swerve;
    private final SwerveKinematicController controller;

    public DriveTrajectoryCommand(Trajectory trajectory, Swerve swerve) {
        this.trajectory = trajectory;
        this.swerve = swerve;
        this.controller = swerve.getController();

        super.addRequirements(swerve);
        
    }
    
}
