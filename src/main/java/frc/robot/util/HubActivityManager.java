package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class HubActivityManager {
  public record GameState(double timeToNextPhase, boolean ourHubEnabled) {}
  ;

  // folowing is copied code from firsts databse that shoudld when called provide
  // weather our alience is active at any point during the match
  // I added the functionality of listing the time untill next hub switch as well

  /***
   * getCurrentHubStatus()
   *
   * @return time till our hub changes again and weather our team's hub is active
   */
  public static GameState getGameState() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    double matchTime = DriverStation.getMatchTime();

    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return new GameState(-1, false);
    }

    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return new GameState(-1, false);
    }

    // We're teleop enabled, compute.
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return new GameState(-1, true);
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return new GameState(-1, true);
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return new GameState(matchTime - 130, true);
    } else if (matchTime > 105) {
      // Shift 1
      return new GameState(matchTime - 105, shift1Active);
    } else if (matchTime > 80) {
      // Shift 2
      return new GameState(matchTime - 80, !shift1Active);
    } else if (matchTime > 55) {
      // Shift 3
      return new GameState(matchTime - 55, shift1Active);
    } else if (matchTime > 30) {
      // Shift 4
      return new GameState(matchTime - 30, !shift1Active);
    } else {
      // End game, hub always active.
      return new GameState(matchTime, true);
    }
  }
}
