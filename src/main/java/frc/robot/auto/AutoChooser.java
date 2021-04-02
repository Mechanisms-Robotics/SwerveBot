package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.Map;

public class AutoChooser {

  private Swerve swerve = null;

  public AutoChooser(Swerve swerve) {
    this.swerve = swerve;
  }

  private Map<String, AutoCommand> autoChoices =
      new HashMap<String, AutoCommand>() {
        {
          put("None", null);
          put("Slalom", new Slalom(swerve));
        }
      };

  public AutoCommand getAuto(String choice) {
    if (choice != null) {
      return this.autoChoices.get(choice);
    }

    return autoChoices.get("None");
  }

  public SendableChooser<AutoCommand> getAutoChooser() {
    SendableChooser<AutoCommand> chooser = new SendableChooser<>();
    chooser.setDefaultOption("SELECT AUTO!", null);

    for (Map.Entry<String, AutoCommand> autoChoice : autoChoices.entrySet()) {
      chooser.addOption(autoChoice.getKey(), autoChoice.getValue());
    }

    chooser.getSelected();
    return chooser;
  }
}
