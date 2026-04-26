// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

  private final PIDController pivotVelocityFeedback =
      IntakeConstants.pivotVelocityControl.pidController(Radians);
  private final ArmFeedforward pivotVelocityFeedforward =
      IntakeConstants.pivotVelocityControl.armFeedforward(Radians);

  private final PIDController pivotPositionFeedback =
      IntakeConstants.pivotPositionControl.pidController(Radians);

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
  }

  private void setRollerVelocity(AngularVelocity velocity) {
    rollerIO.setAngularVelocity(velocity);
    Logger.recordOutput("Intake/Roller/SetpointSpeed", velocity);
  }

  private void coastRoller() {
    rollerIO.coast();
    Logger.recordOutput("Intake/Roller/SetpointSpeed", RadiansPerSecond.of(0));
  }

  private void runPivotAtSpeed(AngularVelocity speed) {
    pivotIO.runOpenLoop(
        pivotVelocityFeedback.calculate(
                pivotInputs.absoluteEncoderVelocity.in(RadiansPerSecond),
                speed.in(RadiansPerSecond))
            + pivotVelocityFeedforward.calculate(
                pivotInputs.absoluteEncoderPosition.in(Radians), speed.in(RadiansPerSecond)));
  }

  private void coastPivot() {
    pivotIO.runOpenLoop(0);
  }

  public Command intakeCommand() {
    return Commands.runEnd(
            () -> {
              if (pivotInputs.absoluteEncoderPosition.lt(
                  IntakeConstants.pivotMinRotation.plus(IntakeConstants.pivotIntakeTolerance))) {
                setRollerVelocity(IntakeConstants.rollerIntakeSpeed);
              } else {
                coastRoller();
              }
            },
            this::coastRoller)
        .alongWith(lowerPivotCommand());
  }

  public Command outtakeCommand() {
    return runEnd(
        () -> setRollerVelocity(IntakeConstants.rollerIntakeSpeed.unaryMinus()), this::coastRoller);
  }

  private Command dependencyFreeAgitateCommand() {
    return Commands.runEnd(
        () -> setRollerVelocity(IntakeConstants.rollerAgitateSpeed), this::coastRoller);
  }

  public Command agitateCommand() {
    Command command = dependencyFreeAgitateCommand();
    command.addRequirements(this);
    return command;
  }

  private Trigger pivotRaised() {
    return new Trigger(
        () ->
            pivotInputs.absoluteEncoderPosition.isNear(
                IntakeConstants.pivotRaiseTargetAngle, IntakeConstants.pivotPositioningTolerance));
  }

  public Command raisePivotCommand() {
    Trigger pivotRaised = pivotRaised();
    return runOnce(() -> pivotPositionFeedback.reset())
        .andThen(
            runEnd(
                () -> {
                  if (pivotRaised.getAsBoolean()) coastPivot();
                  else
                    runPivotAtSpeed(
                        RadiansPerSecond.of(
                            pivotPositionFeedback.calculate(
                                pivotInputs.absoluteEncoderPosition.in(Radians),
                                IntakeConstants.pivotRaiseTargetAngle.in(Radians))));
                },
                this::coastPivot));
  }

  private Trigger pivotLowered() {
    return new Trigger(
        () ->
            pivotInputs.absoluteEncoderPosition.isNear(
                IntakeConstants.pivotMinRotation, IntakeConstants.pivotPositioningTolerance));
  }

  public Command lowerPivotCommand() {
    Trigger pivotLowered = pivotLowered();
    return runOnce(() -> pivotPositionFeedback.reset())
        .andThen(
            runEnd(
                () -> {
                  if (pivotLowered.getAsBoolean()) coastPivot();
                  else
                    runPivotAtSpeed(
                        RadiansPerSecond.of(
                            pivotPositionFeedback.calculate(
                                pivotInputs.absoluteEncoderPosition.in(Radians),
                                IntakeConstants.pivotMinRotation.in(Radians))));
                },
                this::coastPivot));
  }

  public Command raiseWhileAgitating() {
    return raisePivotCommand().alongWith(dependencyFreeAgitateCommand());
  }

  public Command lowerWhileAgitating() {
    return lowerPivotCommand().alongWith(dependencyFreeAgitateCommand());
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
        Volts.of(0.6).per(Second),
        Volts.of(3),
        Seconds.of(8),
        voltage -> pivotIO.runOpenLoop(voltage.in(Volts)),
        () -> pivotInputs.absoluteEncoderPosition,
        () -> pivotInputs.absoluteEncoderVelocity,
        () -> pivotInputs.appliedVoltage,
        this);
  }
}
