package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Per;

public class ControlConstantsBuilder<MeasurementUnit extends Unit, OutputUnit extends Unit> {
  public record ControlConstants(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  public Per<OutputUnit, MeasurementUnit> kP;
  public Per<OutputUnit, MultUnit<MeasurementUnit, TimeUnit>> kI;
  public Per<OutputUnit, PerUnit<MeasurementUnit, TimeUnit>> kD;

  public Measure<OutputUnit> kS;
  public Per<OutputUnit, PerUnit<MeasurementUnit, TimeUnit>> kV;
  public Per<OutputUnit, PerUnit<PerUnit<MeasurementUnit, TimeUnit>, TimeUnit>> kA;

  private MeasurementUnit initialMeasurementUnit;
  private OutputUnit initialOutputUnit;
  private TimeUnit initialTimeUnit;

  @SuppressWarnings("unchecked")
  private ControlConstantsBuilder(
      MeasurementUnit initialMeasurementUnit,
      OutputUnit initialOutputUnit,
      TimeUnit initialTimeUnit) {
    this.initialMeasurementUnit = initialMeasurementUnit;
    this.initialOutputUnit = initialOutputUnit;
    this.initialTimeUnit = initialTimeUnit;

    kP = PerUnit.combine(initialOutputUnit, initialMeasurementUnit).ofNative(0);
    kI =
        PerUnit.combine(
                initialOutputUnit, MultUnit.combine(initialMeasurementUnit, initialTimeUnit))
            .ofNative(0);
    kD =
        PerUnit.combine(initialOutputUnit, PerUnit.combine(initialMeasurementUnit, initialTimeUnit))
            .ofNative(0);
    kS = (Measure<OutputUnit>) initialOutputUnit.zero();
    kV =
        PerUnit.combine(initialOutputUnit, PerUnit.combine(initialMeasurementUnit, initialTimeUnit))
            .ofNative(0);
    kA =
        PerUnit.combine(
                initialOutputUnit,
                PerUnit.combine(
                    PerUnit.combine(initialMeasurementUnit, initialTimeUnit), initialTimeUnit))
            .ofNative(0);
  }

  public ControlConstantsBuilder<MeasurementUnit, OutputUnit> pid(double kP, double kI, double kD) {
    this.kP = PerUnit.combine(initialOutputUnit, initialMeasurementUnit).ofNative(kP);
    this.kI =
        PerUnit.combine(
                initialOutputUnit, MultUnit.combine(initialMeasurementUnit, initialTimeUnit))
            .ofNative(kI);
    this.kD =
        PerUnit.combine(initialOutputUnit, PerUnit.combine(initialMeasurementUnit, initialTimeUnit))
            .ofNative(kD);

    return this;
  }

  @SuppressWarnings("unchecked")
  public ControlConstantsBuilder<MeasurementUnit, OutputUnit> sva(double kS, double kV, double kA) {
    this.kS = (Measure<OutputUnit>) initialOutputUnit.of(kS);
    this.kV =
        PerUnit.combine(initialOutputUnit, PerUnit.combine(initialMeasurementUnit, initialTimeUnit))
            .ofNative(kV);
    this.kA =
        PerUnit.combine(
                initialOutputUnit,
                PerUnit.combine(
                    PerUnit.combine(initialMeasurementUnit, initialTimeUnit), initialTimeUnit))
            .ofNative(kA);

    return this;
  }

  public ControlConstants in(
      MeasurementUnit finalMeasurementUnit, OutputUnit finalOutputUnit, TimeUnit finalTimeUnit) {
    return new ControlConstants(
        kP.in(PerUnit.combine(finalOutputUnit, finalMeasurementUnit)),
        kI.in(
            PerUnit.combine(
                finalOutputUnit, MultUnit.combine(finalMeasurementUnit, finalTimeUnit))),
        kD.in(
            PerUnit.combine(finalOutputUnit, PerUnit.combine(finalMeasurementUnit, finalTimeUnit))),
        kS.in(finalOutputUnit),
        kV.in(
            PerUnit.combine(finalOutputUnit, PerUnit.combine(finalMeasurementUnit, finalTimeUnit))),
        kA.in(
            PerUnit.combine(
                finalOutputUnit,
                PerUnit.combine(
                    PerUnit.combine(finalMeasurementUnit, finalTimeUnit), finalTimeUnit))));
  }

  public static <MeasurementUnit extends Unit, OutputUnit extends Unit>
      ControlConstantsBuilder<MeasurementUnit, OutputUnit> fromUnits(
          MeasurementUnit initialMeasurementUnit,
          OutputUnit initialOutputUnit,
          TimeUnit initialTimeUnit) {
    return new ControlConstantsBuilder<MeasurementUnit, OutputUnit>(
        initialMeasurementUnit, initialOutputUnit, initialTimeUnit);
  }
}
