package frc.robot.util;

import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumble {
    private static final Set<Integer> bothRumbleMatchTimes =
      Set.of(135, 130, 85, 80, 60, 55, 35, 30, 5);
  private static final Set<Integer> leftRumbleMatchTimes = Set.of(120, 95, 70, 45);
  private static final Set<Integer> rightRumbleMatchTimes = Set.of(115, 90, 65, 40);

  public static void rumbleController(CommandXboxController primaryController, CommandXboxController secondaryController) {
    if (bothRumbleMatchTimes.contains((int) DriverStation.getMatchTime())) {
      primaryController.setRumble(RumbleType.kBothRumble, 1);
      secondaryController.setRumble(RumbleType.kBothRumble, 1);
    } else if (leftRumbleMatchTimes.contains((int) DriverStation.getMatchTime())) {
      primaryController.setRumble(RumbleType.kLeftRumble, 1);
      secondaryController.setRumble(RumbleType.kLeftRumble, 1);
    } else if (rightRumbleMatchTimes.contains((int) DriverStation.getMatchTime())) {
      primaryController.setRumble(RumbleType.kRightRumble, 1);
      secondaryController.setRumble(RumbleType.kRightRumble, 1);
    } else {
        primaryController.setRumble(RumbleType.kBothRumble, 0);
        secondaryController.setRumble(RumbleType.kBothRumble, 0);
    }
  }
}
