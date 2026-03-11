package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Set;

public class ControllerRumble {
  private static final Set<Integer> bothRumbleMatchTimes =
      Set.of(135, 130, 110, 105, 85, 80, 60, 55, 35, 30, 5);
  private static final Set<Integer> leftRumbleMatchTimes = Set.of(120, 95, 70, 45, 15);
  private static final Set<Integer> rightRumbleMatchTimes = Set.of(115, 90, 65, 40, 10);

  public static void rumbleController(
      CommandXboxController primaryController, CommandXboxController secondaryController) {
    if (bothRumbleMatchTimes.contains((int) DriverStation.getMatchTime())) {
      primaryController.setRumble(RumbleType.kBothRumble, 0.5);
      secondaryController.setRumble(RumbleType.kBothRumble, 0.5);
    } else if (leftRumbleMatchTimes.contains((int) DriverStation.getMatchTime())) {
      primaryController.setRumble(RumbleType.kLeftRumble, 0.5);
      secondaryController.setRumble(RumbleType.kLeftRumble, 0.5);
    } else if (rightRumbleMatchTimes.contains((int) DriverStation.getMatchTime())) {
      primaryController.setRumble(RumbleType.kRightRumble, 0.5);
      secondaryController.setRumble(RumbleType.kRightRumble, 0.5);
    } else {
      primaryController.setRumble(RumbleType.kBothRumble, 0);
      secondaryController.setRumble(RumbleType.kBothRumble, 0);
    }
  }
}
