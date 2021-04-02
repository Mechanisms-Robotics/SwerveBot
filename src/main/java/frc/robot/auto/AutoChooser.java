package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.Map;

public class AutoChooser {

  private final Swerve swerve;

  private final Map<String, AutoCommand> autoChoices;

  private final SendableChooser<AutoCommand> chooser = new SendableChooser<>();

  public AutoChooser(Swerve swerve) {
    this.swerve = swerve;

    this.autoChoices =
        new HashMap<String, AutoCommand>() {
          {
            put("None", null);
            put("Slalom", new Slalom(swerve));
          }
        };

    this.chooser.setDefaultOption("SELECT AUTO!", null);

    for (Map.Entry<String, AutoCommand> autoChoice : autoChoices.entrySet()) {
      this.chooser.addOption(autoChoice.getKey(), autoChoice.getValue());
    }

    this.chooser.getSelected();
  }

  public SendableChooser<AutoCommand> getAutoChooser() {
    return this.chooser;
  }
}
