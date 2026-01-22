// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IntakeConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(RollerIO rollerIO) {
    this.rollerIO = rollerIO;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);
  }

  public Command runAtSpeedCommand(AngularVelocity velocity) {
    return runEnd(
        () -> rollerIO.setAngularVelocity(velocity),
        () -> rollerIO.setAngularVelocity(RadiansPerSecond.of(0)));
  }

  public Command outtakeCommand() {
    return runAtSpeedCommand(IntakeConstants.rollerOuttakeSpeed.times(-1));
  }

  public Command runAtIntakeSpeedCommand(Supplier<LinearVelocity> drivetrainVelocity) {
    return runEnd(
        () -> {
          AngularVelocity offsetDriveVelocity =
              RadiansPerSecond.of(
                  drivetrainVelocity
                      .get()
                      .div(IntakeConstants.rollerRadius)
                      .in(MetersPerSecond.per(Meters)));
          AngularVelocity velocity = offsetDriveVelocity.plus(IntakeConstants.rollerAbsoluteSpeed);

          rollerIO.setAngularVelocity(velocity);
          Logger.recordOutput("Intake/AbsoluteSetpointSpeed", velocity);
        },
        () -> {
          rollerIO.setAngularVelocity(RadiansPerSecond.of(0));
          Logger.recordOutput("Intake/AbsoluteSetpointSpeed", RadiansPerSecond.of(0));
        });
  }

  //
  // SysId
  //

  public Command sysIdCommand(Trigger advanceRoutine) {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                IntakeConstants.sysIdRampRate,
                IntakeConstants.sysIdStepVoltage,
                IntakeConstants.sysIdTimeout,
                (state) -> {
                  Logger.recordOutput("Intake/SysId/State", state.toString());

                  Logger.recordOutput(
                      "Intake/SysId/PositionRadians", rollerInputs.position.in(Radians));
                  Logger.recordOutput(
                      "Intake/SysId/VelocityRadiansPerSecond",
                      rollerInputs.velocity.in(RadiansPerSecond));
                  Logger.recordOutput(
                      "Intake/SysId/AppliedVolts", rollerInputs.appliedVoltage.in(Volts));
                }),
            new SysIdRoutine.Mechanism(
                (voltage) -> rollerIO.runOpenLoop(voltage.in(Volts)), null, this));

    Supplier<Command> waitCommand =
        () ->
            Commands.parallel(
                Commands.waitUntil(advanceRoutine.negate()),
                Commands.waitSeconds(0.5),
                runOnce(() -> rollerIO.runOpenLoop(0)));

    return Commands.sequence(
        // TODO: the drivetrain sysid routine looks the exact same. remove code repetition
        waitCommand.get(),
        routine.dynamic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
        waitCommand.get(),
        routine.dynamic(SysIdRoutine.Direction.kReverse).until(advanceRoutine),
        waitCommand.get(),
        routine.quasistatic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
        waitCommand.get(),
        routine.quasistatic(SysIdRoutine.Direction.kReverse).until(advanceRoutine));
  }
}
