package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.choreo.ChoreoVars;
import frc.robot.util.ControlConstantsBuilder;

public class SwerveConstants {
  public static final double teleopJoystickDeadband = 0.03;

  public static final LinearVelocity linearTeleopSpeed = MetersPerSecond.of(3);
  public static final AngularVelocity angularTeleopSpeed = RotationsPerSecond.of(0.5);

  public static final LinearVelocity linearTeleopSpeedWhileShooting = MetersPerSecond.of(1);

  public static final Angle rotationToleranceBeforeShooting = Degrees.of(5);

  // control
  public static final Angle rotationFeedbackTolerance = Degrees.of(1.5);
  public static final ControlConstantsBuilder rotationControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(5.0, 0, 0.3)
          .iZone(1)
          .constraints(RotationsPerSecond.of(3), RotationsPerSecondPerSecond.of(9));

  public static final PIDConstants driveControl = new PIDConstants(4.2, 0, 0);

  // pathfinding
  public static final Distance trenchTraversalOffset = Meters.of(2);
  public static final PathConstraints automaticsConstraints =
      new PathConstraints(
          MetersPerSecond.of(2),
          MetersPerSecondPerSecond.of(4),
          RotationsPerSecond.of(1),
          RotationsPerSecondPerSecond.of(2));

  // physical properties
  public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(5.14);

  public static final Distance driveBaseRadius =
      Meters.of(
          Math.hypot(
              ChoreoVars.R_TrackLength.div(2).in(Meters),
              ChoreoVars.R_TrackLength.div(2).in(Meters)));

  // odometry frequency
  public static final Frequency canFdOdometryFrequency = Hertz.of(250);
  public static final Frequency baseOdometryFrequency = Hertz.of(100);

  public static final Frequency odometryFrequency() {
    return swerveCANBus.isNetworkFD() ? canFdOdometryFrequency : baseOdometryFrequency;
  }

  // misc
  public static final CANBus swerveCANBus = CANBus.roboRIO();

  public static final Time idleTimeUntilSetpointGeneratorReset = Milliseconds.of(60);
}
