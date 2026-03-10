// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.GeneralPurposeCharacterization;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(RollerIO rollerIO, PivotIO pivotIO) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
  }

  private void setRollerVelocity(AngularVelocity velocity) {
    rollerIO.setAngularVelocity(velocity);
    Logger.recordOutput("Intake/Roller/SetpointSpeed", velocity);
  }

  private void coastRoller() {
    rollerIO.coast();
    Logger.recordOutput("Intake/Roller/SetpointSpeed", RadiansPerSecond.of(0));
  }

  private void coastPivot() {
    pivotIO.runOpenLoop(0);
  }

  public Command intakeCommand() {
    return Commands.runEnd(
            () -> setRollerVelocity(IntakeConstants.rollerIntakeSpeed), this::coastRoller)
        .alongWith(lowerPivotCommand());
  }

  public Command outtakeCommand() {
    return runEnd(
        () -> setRollerVelocity(IntakeConstants.rollerIntakeSpeed.unaryMinus()), this::coastRoller);
  }

  public Command agitateCommand() {
    return runEnd(() -> setRollerVelocity(IntakeConstants.rollerAgitateSpeed), this::coastRoller);
  }

  public Command raisePivotCommand() {
    return runEnd(
            () -> pivotIO.runOpenLoop(IntakeConstants.pivotOpenLoopVolts.in(Volts)),
            this::coastPivot)
        .withDeadline(
            Commands.sequence(
                Commands.waitSeconds(0.05), // TODO: constants. same with the 0.8
                Commands.waitUntil(
                    () ->
                        pivotInputs.appliedVoltage.abs(Volts)
                            < IntakeConstants.pivotOpenLoopVolts.times(0.8).abs(Volts))));
  }

  public Command lowerPivotCommand() {
    return runEnd(
            () -> pivotIO.runOpenLoop(IntakeConstants.pivotOpenLoopVolts.unaryMinus().in(Volts)),
            this::coastPivot)
        .withDeadline(
            Commands.sequence(
                Commands.waitSeconds(0.05), // TODO: constants. same with the 0.8
                Commands.waitUntil(
                    () ->
                        pivotInputs.appliedVoltage.abs(Volts)
                            < IntakeConstants.pivotOpenLoopVolts.times(0.8).abs(Volts))));
  }

  public Command runPivotOpenLoopCommand(Voltage output) {
    return runEnd(() -> pivotIO.runOpenLoop(output.in(Volts)), this::coastPivot);
  }

  public Angle getPivotAngle() {
    return pivotInputs.angle;
  }

  //
  // SysId
  //
  public Command rollerSysIdCommand(Trigger advanceRoutine) {
    return GeneralPurposeCharacterization.sysIdCommand(
        advanceRoutine,
        "Intake/SysId/Roller",
        voltage -> rollerIO.runOpenLoop(voltage.in(Volts)),
        () -> rollerInputs.position,
        () -> rollerInputs.velocity,
        () -> rollerInputs.appliedVoltage,
        this);
  }

  public Command pivotSysIdCommand(Trigger advanceRoutine) {
    return GeneralPurposeCharacterization.sysIdCommand(
        advanceRoutine,
        "Intake/SysId/Pivot",
        Volts.of(0.4).per(Second),
        Volts.of(1.2),
        Seconds.of(8),
        voltage -> pivotIO.runOpenLoop(voltage.in(Volts)),
        () -> pivotInputs.angle,
        () -> pivotInputs.angularVelocity,
        () -> pivotInputs.appliedVoltage,
        this);
  }
}
