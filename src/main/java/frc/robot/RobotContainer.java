// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer {

  // Subsystems
  // private final Swerve swerve = new Swerve();
  private final Shooter shooter = new Shooter();
  private final Accelerator accelerator = new Accelerator();
  private final Spindexer spindexer = new Spindexer();
  private final Climber climber = new Climber();
  
  // The driver's controller
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final Button shootTrigger = new Button(() -> driverController.getRawAxis(3) > 0.1);

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();

    // swerve.zeroHeading();
  }

  private void configureButtonBindings() {
    shootTrigger.toggleWhenPressed(
        new ShootCommand(
            () -> driverController.getBumper(Hand.kRight),
            () -> driverController.getBumper(Hand.kLeft),
            shooter,
            accelerator,
            spindexer));
  }

  private void configureDefaultCommands() {
    // Drive the robot relative to the field\
    /*
    swerve.setDefaultCommand(
        new DriveTeleopCommand(
            () -> driverController.getX(Hand.kLeft),
            () -> -driverController.getY(Hand.kLeft),
            () -> driverController.getX(Hand.kRight),
            swerve));
     */
    // spindexer.setDefaultCommand(
    //        new RunCommand(
    //                () -> spindexer.setOpenLoop(0.15)
    //        )
    // );
    climber.setDefaultCommand(new ClimberCommand(() -> operatorController.getRawAxis(5), climber));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
