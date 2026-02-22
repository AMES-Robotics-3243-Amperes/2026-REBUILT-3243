package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.choreo.ChoreoVars;
import frc.robot.util.ControlConstantsBuilder;

public class SwerveConstants {
  // teleop
  public static final double teleopJoystickDeadband = 0;

  public static final LinearVelocity linearTeleopSpeed = MetersPerSecond.of(3);
  public static final AngularVelocity angularTeleopSpeed = RotationsPerSecond.of(0.5);

  public static final LinearVelocity linearTeleopSpeedWhileShooting = MetersPerSecond.of(0.5);

  // control
  public static final Angle rotationFeedBackTolerance = Degrees.of(0.6);
  public static final ControlConstantsBuilder rotationControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(4.8, 0, 0.07)
          .iZone(1)
          .constraints(RotationsPerSecond.of(3), RotationsPerSecondPerSecond.of(9));

  public static final PIDConstants driveControl = new PIDConstants(5, 0, 0);

  // physical properties
  public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(5.14);
  public static final Current driveCurrentLimit = Amps.of(120);

  public static final Distance driveBaseRadius =
      Meters.of(
          Math.hypot(
              ChoreoVars.R_TrackLength.div(2).in(Meters),
              ChoreoVars.R_TrackLength.div(2).in(Meters)));

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
