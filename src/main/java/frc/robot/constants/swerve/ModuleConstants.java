package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ControlConstantsBuilder;

public class ModuleConstants {
  // drive
  public static final ControlConstantsBuilder driveControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(11.5, 0, 0).sva(5.8, 0, 0);

  // turn
  public static final ControlConstantsBuilder steerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(24, 0, 0)
          .constraints(RotationsPerSecond.of(25), RotationsPerSecondPerSecond.of(250));

  public static final Current steerStatorCurrentLimit = Amps.of(30);
  public static final Current driveCurrentLimit = Amps.of(80);

  public static final AngularVelocity maxSetpointGeneratorModuleRotation = RotationsPerSecond.of(5);

  // physical properties. ids and other hardware-specific things go in tuner constants
  public static final double kCoupleRatio = 0;
  public static final double steerReduction = 25;

  public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  public static final ClosedLoopOutputType driveClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;

  public static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // simulation
  public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
  public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.005);

  public static final Voltage driveFrictionVoltage = Volts.of(0.1);
  public static final Voltage steerFrictionVoltage = Volts.of(0.1);
}
