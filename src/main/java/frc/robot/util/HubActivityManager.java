package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class HubActivityManager {
  public record HubState(double timeToNextPhase, boolean ourHubEnabled) {}
  ;

  // following is copied code from first's databse that should when called provide
  // weather our alience is active at any point during the match
  // I added the functionality of listing the time untill next hub switch as well

  public static HubState getHubState() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double matchTime = DriverStation.getMatchTime();

    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return new HubState(-1, false);
    }

    if (DriverStation.isAutonomous()) {
      return new HubState(matchTime, true);
    }

    // We're teleop enabled, compute.
    String gameData = DriverStation.getGameSpecificMessage();

    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return new HubState(matchTime - 130, true);
    }

    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return new HubState(-1, true);
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    SmartDashboard.putBoolean("Won Auto", !shift1Active);

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return new HubState(matchTime - 130, true);
    } else if (matchTime > 105) {
      // Shift 1
      return new HubState(matchTime - 105, shift1Active);
    } else if (matchTime > 80) {
      // Shift 2
      return new HubState(matchTime - 80, !shift1Active);
    } else if (matchTime > 55) {
      // Shift 3
      return new HubState(matchTime - 55, shift1Active);
    } else if (matchTime > 30) {
      // Shift 4
      return new HubState(matchTime - 30, !shift1Active);
    } else {
      // End game, hub always active.
      return new HubState(matchTime, true);
    }
  }
}
