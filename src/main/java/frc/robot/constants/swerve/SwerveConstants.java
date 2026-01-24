package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.PIDBuilder;

public class SwerveConstants {
  // teleop
  public static final double teleopJoystickDeadband = 0.3;
  public static final double teleopAbsoluteRotationDeadband = 0.5;

  public static final LinearVelocity linearTeleopSpeed = MetersPerSecond.of(2);
  public static final AngularVelocity angularTeleopSpeed = RotationsPerSecond.of(0.5);

  // holonomic pid controllers. note that any configuration beyond kP, kI, and kD will NOT apply to
  // pathplanner autos
  public static final PIDBuilder drivePid = new PIDBuilder().pid(3, 0, 0).iZone(1);
  public static final PIDBuilder rotationPidRadians =
      new PIDBuilder().pid(8, 0, 0.05).constraints(12, 30).iZone(1);

  // physical properties
  public static final Distance driveBaseFrontLength = Inches.of(26);
  public static final Distance driveBaseSideLength = Inches.of(26);

  public static final Mass robotMass = Pounds.of(120);
  public static final MomentOfInertia robotMomentOfInertia = KilogramSquareMeters.of(6);
  public static final double wheelCoefficientOfFriction = 1.0;
  public static final Current slipCurrent = Amps.of(120);
  public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(5);

  public static final Distance driveBaseRadius =
      Meters.of(
          Math.hypot(
              driveBaseFrontLength.div(2).in(Meters), driveBaseSideLength.div(2).in(Meters)));

  // odometry frequency
  public static final Frequency canFdOdometryFrequency = Hertz.of(250);
  public static final Frequency baseOdometryFrequency = Hertz.of(120);

  public static final Frequency odometryFrequency() {
    return swerveCANBus.isNetworkFD() ? canFdOdometryFrequency : baseOdometryFrequency;
  }

  // misc
  public static final CANBus swerveCANBus = CANBus.roboRIO();

  public static final Time idleTimeUntilSetpointGeneratorReset = Milliseconds.of(60);
}
