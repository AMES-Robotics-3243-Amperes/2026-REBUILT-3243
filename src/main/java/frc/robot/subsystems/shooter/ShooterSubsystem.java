// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
  }

  public void setShooterVelocity(AngularVelocity velocity) {
    shooterIO.setShooterAngularVelocity(velocity);
    Logger.recordOutput("Shooter/SetpointVelocity", velocity);
  }

  public void setHoodAngle(Angle angle) {
    shooterIO.setHoodAngle(angle);
    Logger.recordOutput("Shooter/SetpointHoodAngle", angle);
  }

  public void stopShooter() {
    shooterIO.runShooterOpenLoop(0);
    Logger.recordOutput("Shooter/SetpointVelocity", RPM.of(0));
  }

  public void stopHood() {
    shooterIO.runHoodOpenLoop(0);
  }
}
