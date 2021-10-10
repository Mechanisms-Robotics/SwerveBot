package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Util {
  public static Rotation2d getTranslationDirection(Translation2d translation) {
    return new Rotation2d(translation.getX(), translation.getY());
  }

  public static Rotation2d nearestPole(Rotation2d rotation2d) {
    double poleSin = 0.0;
    double poleCos = 0.0;
    if (Math.abs(rotation2d.getCos()) > Math.abs(rotation2d.getSin())) {
      poleCos = Math.signum(rotation2d.getCos());
      poleSin = 0.0;
    } else {
      poleCos = 0.0;
      poleSin = Math.signum(rotation2d.getSin());
    }
    return new Rotation2d(poleCos, poleSin);
  }
}
