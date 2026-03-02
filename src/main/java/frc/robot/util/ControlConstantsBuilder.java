package frc.robot.util;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import java.util.Optional;

public class ControlConstantsBuilder {
  public record ControlConstants(
      double kP,
      double kI,
      double kD,
      double iZone,
      double kS,
      double kV,
      double kA,
      double kG,
      boolean cosCompensatedKg,
      Optional<Double> maxVelocity,
      Optional<Double> maxAcceleration) {}

  public Per<DimensionlessUnit, AngleUnit> kP = Value.per(Radian).ofNative(0);
  public Per<DimensionlessUnit, MultUnit<AngleUnit, TimeUnit>> kI =
      Value.per(MultUnit.combine(Radian, Second)).ofNative(0);
  public Per<DimensionlessUnit, AngularVelocityUnit> kD = Value.per(RadiansPerSecond).ofNative(0);

  public double iZone = 1;

  public double kS = 0;
  public Per<DimensionlessUnit, AngularVelocityUnit> kV = Value.per(RadiansPerSecond).ofNative(0);
  public Per<DimensionlessUnit, AngularAccelerationUnit> kA =
      Value.per(RadiansPerSecondPerSecond).ofNative(0);

  public double kG = 0;
  public boolean cosCompensatedKg = false;

  public Optional<AngularVelocity> maxVelocity = Optional.empty();
  public Optional<AngularAcceleration> maxAcceleration = Optional.empty();

  //
  // Initialization
  //

  private ControlConstantsBuilder() {}

  public static <MeasurementUnit extends Unit, OutputUnit extends Unit>
      ControlConstantsBuilder fromRadiansAndSeconds() {
    return new ControlConstantsBuilder();
  }

  public ControlConstantsBuilder pid(double kP, double kI, double kD) {
    this.kP = Value.per(Radian).ofNative(kP);
    this.kI = Value.per(MultUnit.combine(Radian, Second)).ofNative(kI);
    this.kD = Value.per(RadiansPerSecond).ofNative(kD);

    return this;
  }

  public ControlConstantsBuilder sva(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = Value.per(RadiansPerSecond).ofNative(kV);
    this.kA = Value.per(RadiansPerSecondPerSecond).ofNative(kA);

    return this;
  }

  public ControlConstantsBuilder g(double kG, boolean cosCompensatedKg) {
    this.kG = kG;
    this.cosCompensatedKg = cosCompensatedKg;

    return this;
  }

  public ControlConstantsBuilder constraints(
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    this.maxVelocity = Optional.of(maxVelocity);
    this.maxAcceleration = Optional.of(maxAcceleration);

    return this;
  }

  public ControlConstantsBuilder iZone(double iZone) {
    this.iZone = iZone;

    return this;
  }

  public PIDConstants pathPlannerPIDConstants() {
    ControlConstants constants = in(Radians, Seconds);
    return new PIDConstants(constants.kP(), constants.kI(), constants.kD(), iZone);
  }

  //
  // Construction
  //

  public ControlConstants in(AngleUnit angleUnit, TimeUnit timeUnit) {
    return new ControlConstants(
        kP.in(Value.per(angleUnit)),
        kI.in(Value.per(MultUnit.combine(angleUnit, timeUnit))),
        kD.in(Value.per(angleUnit.per(timeUnit))),
        iZone,
        kS,
        kV.in(Value.per(angleUnit.per(timeUnit))),
        kA.in(Value.per(angleUnit.per(timeUnit).per(timeUnit))),
        kG,
        cosCompensatedKg,
        maxVelocity.map(velocity -> velocity.in(angleUnit.per(timeUnit))),
        maxAcceleration.map(
            acceleration -> acceleration.in(angleUnit.per(timeUnit).per(timeUnit))));
  }

  public PIDController pidController(AngleUnit angleUnit, Angle minWrap, Angle maxWrap) {
    ControlConstants constants = in(angleUnit, Seconds);

    PIDController controller = new PIDController(constants.kP(), constants.kI(), constants.kD());

    controller.setIZone(iZone);
    controller.enableContinuousInput(minWrap.in(angleUnit), maxWrap.in(angleUnit));

    return controller;
  }

  public PIDController pidController(AngleUnit angleUnit) {
    ControlConstants constants = in(angleUnit, Seconds);

    PIDController controller = new PIDController(constants.kP(), constants.kI(), constants.kD());
    controller.setIZone(iZone);

    return controller;
  }

  public ProfiledPIDController profiledPIDController(
      AngleUnit angleUnit, Angle minWrap, Angle maxWrap) {
    ControlConstants constants = in(angleUnit, Seconds);

    double maxVelocity =
        this.maxVelocity
            .orElse(RotationsPerSecond.of(Double.POSITIVE_INFINITY))
            .in(angleUnit.per(Second));
    double maxAcceleration =
        this.maxAcceleration
            .orElse(RotationsPerSecondPerSecond.of(Double.POSITIVE_INFINITY))
            .in(angleUnit.per(Second).per(Second));

    ProfiledPIDController controller =
        new ProfiledPIDController(
            constants.kP(),
            constants.kI(),
            constants.kD(),
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    controller.setIZone(iZone);
    controller.enableContinuousInput(minWrap.in(angleUnit), maxWrap.in(angleUnit));

    return controller;
  }

  public ProfiledPIDController profiledPIDController(AngleUnit angleUnit) {
    ControlConstants constants = in(angleUnit, Seconds);

    double maxVelocity =
        this.maxVelocity
            .orElse(RotationsPerSecond.of(Double.POSITIVE_INFINITY))
            .in(angleUnit.per(Second));
    double maxAcceleration =
        this.maxAcceleration
            .orElse(RotationsPerSecondPerSecond.of(Double.POSITIVE_INFINITY))
            .in(angleUnit.per(Second).per(Second));

    ProfiledPIDController controller =
        new ProfiledPIDController(
            constants.kP(),
            constants.kI(),
            constants.kD(),
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    controller.setIZone(iZone);

    return controller;
  }

  public ClosedLoopConfig revClosedLoopConfig() {
    ClosedLoopConfig config = new ClosedLoopConfig();
    ControlConstants constantsRotationsMs = in(Rotations, Milliseconds);

    config
        .pid(constantsRotationsMs.kP(), constantsRotationsMs.kI(), constantsRotationsMs.kD())
        .iZone(iZone);

    if (kS != 0) config.feedForward.kS(kS);
    if (kV.in(Value.per(RPM)) != 0) config.feedForward.kV(kV.in(Value.per(RPM)));
    if (kA.in(Value.per(RPM.per(Second))) != 0)
      config.feedForward.kA(kA.in(Value.per(RPM.per(Second))));

    if (kG != 0 && cosCompensatedKg) config.feedForward.kCos(kG);
    if (kG != 0 && !cosCompensatedKg) config.feedForward.kG(kG);

    maxVelocity.ifPresent(velocity -> config.maxMotion.cruiseVelocity(velocity.in(RPM)));
    maxAcceleration.ifPresent(
        acceleration -> config.maxMotion.maxAcceleration(acceleration.in(RPM.per(Second))));

    return config;
  }

  public Pair<SlotConfigs, MotionMagicConfigs> talonFXConfigs() {
    ControlConstants constantsRotationsSeconds = in(Rotations, Seconds);

    SlotConfigs slotConfigs =
        new SlotConfigs()
            .withKP(constantsRotationsSeconds.kP())
            .withKI(constantsRotationsSeconds.kI())
            .withKD(constantsRotationsSeconds.kD())
            .withKS(constantsRotationsSeconds.kS())
            .withKV(constantsRotationsSeconds.kV())
            .withKA(constantsRotationsSeconds.kA())
            .withKG(constantsRotationsSeconds.kG())
            .withGravityType(
                cosCompensatedKg ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    maxVelocity.ifPresent(velocity -> motionMagicConfigs.withMotionMagicCruiseVelocity(velocity));
    maxAcceleration.ifPresent(
        acceleration -> motionMagicConfigs.withMotionMagicAcceleration(acceleration));

    return new Pair<SlotConfigs, MotionMagicConfigs>(slotConfigs, motionMagicConfigs);
  }
}
