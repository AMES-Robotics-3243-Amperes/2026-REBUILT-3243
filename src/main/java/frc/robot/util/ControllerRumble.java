package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Set;

public class ControllerRumble {
  private static final Set<Integer> rumbleTimes = Set.of(115, 90, 65, 40, 10);

  public static void rumbleController(
      CommandXboxController primaryController, CommandXboxController secondaryController) {
    if (rumbleTimes.contains((int) DriverStation.getMatchTime())) {
      primaryController.setRumble(RumbleType.kBothRumble, 0.4);
      secondaryController.setRumble(RumbleType.kBothRumble, 0.4);
    } else {
      primaryController.setRumble(RumbleType.kBothRumble, 0);
      secondaryController.setRumble(RumbleType.kBothRumble, 0);
    }
  }
}
