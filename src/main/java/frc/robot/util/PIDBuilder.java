package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.Optional;

public class PIDBuilder {
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private Optional<Double> iZone = Optional.empty();
  private Optional<Double> tolerance = Optional.empty();
  private Optional<Pair<Double, Double>> wrapping = Optional.empty();

  private TrapezoidProfile.Constraints constraints =
      new Constraints(Double.MAX_VALUE, Double.MAX_VALUE);

  public PIDBuilder() {}

  public PIDBuilder kP(double kP) {
    this.kP = kP;
    return this;
  }

  public PIDBuilder kI(double kI) {
    this.kI = kI;
    return this;
  }

  public PIDBuilder kD(double kD) {
    this.kD = kD;
    return this;
  }

  public PIDBuilder pid(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    return this;
  }

  public PIDBuilder iZone(double iZone) {
    this.iZone = Optional.of(iZone);
    return this;
  }

  public PIDBuilder tolerance(double tolerance) {
    this.tolerance = Optional.of(tolerance);
    return this;
  }

  public PIDBuilder wrapping(double min, double max) {
    this.wrapping = Optional.of(Pair.of(min, max));
    return this;
  }

  /** Only applies when building profiled pid controllers. */
  public PIDBuilder constraints(double maxVelocity, double maxAcceleration) {
    this.constraints = new Constraints(maxVelocity, maxAcceleration);
    return this;
  }

  public PIDController build() {
    PIDController controller = new PIDController(kP, kI, kD);

    iZone.ifPresent(iZone -> controller.setIZone(iZone));
    tolerance.ifPresent(tolerance -> controller.setTolerance(tolerance));
    wrapping.ifPresent(
        wrapping -> controller.enableContinuousInput(wrapping.getFirst(), wrapping.getSecond()));

    return controller;
  }

  public ProfiledPIDController buildProfiled() {
    ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, constraints);

    iZone.ifPresent(iZone -> controller.setIZone(iZone));
    tolerance.ifPresent(tolerance -> controller.setTolerance(tolerance));
    wrapping.ifPresent(
        wrapping -> controller.enableContinuousInput(wrapping.getFirst(), wrapping.getSecond()));

    return controller;
  }

  public PIDConstants buildConstants() {
    return iZone
        .map(iZone -> new PIDConstants(kP, kI, kD, iZone))
        .orElse(new PIDConstants(kP, kI, kD));
  }
}
