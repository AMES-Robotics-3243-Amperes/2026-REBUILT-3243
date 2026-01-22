package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ControlConstantsBuilder;

public class IntakeConstants {
  // left vs right is from the perspective of facing out from the robot's center
  public static final int rollerId = 2;

  public static final double rollerGearRatio = 0.5;

  public static final AngularVelocity rollerAbsoluteSpeed = RotationsPerSecond.of(20);
  public static final AngularVelocity rollerOuttakeSpeed = RotationsPerSecond.of(10);

  public static final Distance rollerRadius = Inches.of(1);

  public static final ControlConstantsBuilder<AngleUnit, VoltageUnit> rollerControl =
      ControlConstantsBuilder.fromUnits(Radians, Volts, Seconds)
          .pid(0.000002, 0, 0)
          .sva(0, 0.032867, 0.012696);

  public static final Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(0.3);
  public static final Voltage sysIdStepVoltage = Volts.of(1.5);
  public static final Time sysIdTimeout = Seconds.of(8);
}
