// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  /**
   * Represents the position of the relative encoder where the pivot is bottomed out and the system
   * at the "edge" of the backlash, i.e. spinning the motor at all will immediately pivot the
   * intake.
   */
  @AutoLogOutput(key = "Intake/Pivot/RelativeEncoderBottom")
  private Angle pivotRelativeEncoderBottom = Rotations.of(0);

  private boolean hasInitilizedRelativeBottom = false;

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

    if (!hasInitilizedRelativeBottom) {
      pivotRelativeEncoderBottom =
          pivotInputs.internalEncoderPosition.minus(pivotInputs.absoluteEncoderPosition);
      hasInitilizedRelativeBottom = true;
    }

    // rezero backlash if necessary - see the comments above these constants for more info
    if (pivotInputs.absoluteEncoderVelocity.gt(IntakeConstants.pivotAbsoluteVelocityForRezero)
        && pivotInputs.absoluteEncoderPosition.lt(
            Degrees.of(90).minus(IntakeConstants.pivotAbsolutePositionToleranceForRezero))) {
      pivotRelativeEncoderBottom =
          pivotInputs.internalEncoderPosition.minus(pivotInputs.absoluteEncoderPosition);
    } else if (pivotInputs.absoluteEncoderVelocity.lt(
            IntakeConstants.pivotAbsoluteVelocityForRezero.unaryMinus())
        && pivotInputs.absoluteEncoderPosition.gt(
            Degrees.of(90).plus(IntakeConstants.pivotAbsolutePositionToleranceForRezero))) {
      pivotRelativeEncoderBottom =
          pivotInputs
              .internalEncoderPosition
              .minus(pivotInputs.absoluteEncoderPosition)
              .plus(IntakeConstants.pivotBacklash);
    }
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
        .alongWith(lowerPivotCommand().withTimeout(2.8));
  }

  public Command outtakeCommand() {
    return runEnd(
        () -> setRollerVelocity(IntakeConstants.rollerIntakeSpeed.unaryMinus()), this::coastRoller);
  }

  public Command agitateCommand() {
    return runEnd(() -> setRollerVelocity(IntakeConstants.rollerAgitateSpeed), this::coastRoller);
  }

  public Command raisePivotCommand() {
    return runEnd(() -> pivotIO.runOpenLoop(0.2), this::coastPivot)
        .withDeadline(
            Commands.waitUntil(
                () ->
                    pivotInputs.absoluteEncoderPosition.isNear(
                        IntakeConstants.pivotMaxRotation,
                        IntakeConstants.pivotPositioningTolerance)))
        .withDeadline(Commands.waitTime(IntakeConstants.pivotControlSafety));
  }

  public Command lowerPivotCommand() {
    return runEnd(() -> pivotIO.runOpenLoop(-0.2), this::coastPivot)
        .withDeadline(
            Commands.waitUntil(
                () ->
                    pivotInputs.absoluteEncoderPosition.isNear(
                        IntakeConstants.pivotMinRotation,
                        IntakeConstants.pivotPositioningTolerance)))
        .withDeadline(Commands.waitTime(IntakeConstants.pivotControlSafety));
  }

  public Command runPivotOpenLoopCommand(Voltage output) {
    return runEnd(() -> pivotIO.runOpenLoop(output.in(Volts)), this::coastPivot);
  }

  public Angle getPivotAngle() {
    return pivotInputs.absoluteEncoderPosition;
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
        () -> pivotInputs.absoluteEncoderPosition,
        () -> pivotInputs.absoluteEncoderVelocity,
        () -> pivotInputs.appliedVoltage,
        this);
  }
}
