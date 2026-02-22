package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(6, 0, 0).sva(1.8, 0, 0);

  // turn
  public static final ControlConstantsBuilder steerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(5, 0, 0).sva(0, 0, 0);

  public static final Current steerStatorCurrentLimit = Amps.of(60);

  public static final AngularVelocity maxSetpointGeneratorModuleRotation = RotationsPerSecond.of(5);

  // physical properties. ids and other hardware-specific things go in tuner constants
  public static final double kCoupleRatio = 0;
  public static final double steerReduction = 25;

  public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  public static final ClosedLoopOutputType driveClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;

  public static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.RemoteCANcoder;

  // simulation
  public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
  public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

  public static final Voltage driveFrictionVoltage = Volts.of(driveControl.kS);
  public static final Voltage steerFrictionVoltage = Volts.of(steerControl.kS);
}
