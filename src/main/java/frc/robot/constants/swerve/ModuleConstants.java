package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ControlConstantsBuilder;

public class ModuleConstants {
  // drive
  public static final ControlConstantsBuilder<AngleUnit, VoltageUnit> driveControl =
      ControlConstantsBuilder.fromUnits(Radians, Volts, Seconds)
          .pid(0.021563, 0, 0)
          .sva(0.02585, 0.12346, 0.010446);

  // turn
  public static final ControlConstantsBuilder<AngleUnit, VoltageUnit> steerControl =
      ControlConstantsBuilder.fromUnits(Radians, Volts, Seconds).pid(5, 1.5, 0.2).sva(0, 0.4, 0);

  public static final Current steerStatorCurrentLimit = Amps.of(60);

  public static final AngularVelocity maxSetpointGeneratorModuleRotation =
      RotationsPerSecond.of(10);

  // physical properties. ids and other hardware-specific things go in tuner constants
  public static final double kCoupleRatio = 0;

  public static final double driveGearRatio = 5.79;
  public static final double steerGearRatio = 25;
  public static final Distance kWheelRadius = Inches.of(1.967);

  public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  public static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.RemoteCANcoder;

  // simulation
  public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
  public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

  public static final Voltage driveFrictionVoltage = Volts.of(driveControl.kS.in(Volts));
  public static final Voltage steerFrictionVoltage = Volts.of(steerControl.kS.in(Volts));
}
