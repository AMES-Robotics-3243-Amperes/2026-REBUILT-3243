package frc.robot.util;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.util.PointOfInterestManager.FlipType;
import java.util.function.Supplier;

public class RobotLocationManager {
  public enum RobotLocation {
    ALLIANCE_ZONE,
    NEUTRAL_ZONE,
    OPPONENT_ZONE,
  }

  private RobotLocation location = RobotLocation.ALLIANCE_ZONE;
  private final Subsystem locationSetRequirement = new Subsystem() {};

  public RobotLocationManager(Supplier<Translation2d> robotPosition) {
    locationSetRequirement.setDefaultCommand(
        locationSetRequirement.run(
            () -> {
              Translation2d position = robotPosition.get();
              Rectangle2d allianceZone =
                  PointOfInterestManager.flipRectangleConditionally(
                      FieldConstants.allianceZone, FlipType.REFLECT_FOR_OTHER_ALLIANCE);
              Rectangle2d opponentZone =
                  PointOfInterestManager.flipRectangle(
                      allianceZone, FlipType.REFLECT_FOR_OTHER_ALLIANCE);

              if (allianceZone.contains(position)) location = RobotLocation.ALLIANCE_ZONE;
              else if (opponentZone.contains(position)) location = RobotLocation.OPPONENT_ZONE;
              else if (FieldConstants.neutralZone.contains(position))
                location = RobotLocation.NEUTRAL_ZONE;
            }));
  }

  public Command setLocationAsAllianceZone() {
    return Commands.startEnd(
        () -> location = RobotLocation.ALLIANCE_ZONE, () -> {}, locationSetRequirement);
  }

  public Command setLocationAsNeutralZone() {
    return Commands.startEnd(
        () -> location = RobotLocation.ALLIANCE_ZONE, () -> {}, locationSetRequirement);
  }

  public Command setLocationAsOpponentZone() {
    return Commands.startEnd(
        () -> location = RobotLocation.ALLIANCE_ZONE, () -> {}, locationSetRequirement);
  }

  public Trigger inAllianceZone() {
    return new Trigger(() -> location == RobotLocation.ALLIANCE_ZONE);
  }

  public Trigger inOpponentZone() {
    return new Trigger(() -> location == RobotLocation.OPPONENT_ZONE);
  }

  public Trigger innNeutralZone() {
    return new Trigger(() -> location == RobotLocation.NEUTRAL_ZONE);
  }

  public RobotLocation robotLocation() {
    return location;
  }
}
