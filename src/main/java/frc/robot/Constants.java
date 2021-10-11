// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int startupCanTimeout = 100; // ms
  public static final int canTimeout = 10; // ms
  public static final double loopTime = 0.01; // s
  public static final int talonPrimaryPid = 0;
  public static final int falconCPR = 2048; // counts per revolution

  // TODO: Change to velocity once PIDs are tuned.
  // Currently open loop precentage
  public static final double shooterShootSpeed = 0.70;
  public static final double acceleratorShootSpeed = 0.70;
  public static final double spindexerShootSpeed = 0.75;
  public static final double spindexerIntakeSpeed = 0.25;
  public static final double spindexerPrepSpeed = 0.15;
  public static final double intakeSpeed = 0.5;
}
