package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IntakeConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class RollerIOSim implements RollerIO {
  AngularVelocity velocity = RadiansPerSecond.of(0);

  public RollerIOSim(IntakeSimulation intakeSimulation, SwerveDriveSimulation driveSimulation) {
    intakeSimulation.setCustomIntakeCondition(
        fuel -> {
          Translation2d robotSpeeds =
              new Translation2d(
                  driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative()
                      .vxMetersPerSecond,
                  driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative()
                      .vyMetersPerSecond);
          Translation2d robotRelativeFuelVelocity =
              fuel.getVelocity3dMPS().toTranslation2d().minus(robotSpeeds);
          double relativeFuelSpeedInRobotDir =
              robotSpeeds.dot(robotRelativeFuelVelocity) / robotSpeeds.getNorm();
          return MetersPerSecond.of(
                  velocity.in(RadiansPerSecond) * IntakeConstants.rollerRadius.in(Meters))
              .plus(MetersPerSecond.of(1))
              .gt(MetersPerSecond.of(relativeFuelSpeedInRobotDir));
        });
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.position = Rotations.of(0);
    inputs.velocity = velocity;
    inputs.appliedVoltage =
        Volts.of(
            velocity.in(RadiansPerSecond)
                * IntakeConstants.rollerControl.kV.in(Value.per(RadiansPerSecond)));
  }

  @Override
  public void runOpenLoop(double output) {
    velocity =
        RadiansPerSecond.of(
            output / IntakeConstants.rollerControl.kV.in(Value.per(RadiansPerSecond)));
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    this.velocity = velocity;
  }
}
