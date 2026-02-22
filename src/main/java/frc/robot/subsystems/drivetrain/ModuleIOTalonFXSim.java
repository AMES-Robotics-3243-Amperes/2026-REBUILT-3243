// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import frc.robot.constants.swerve.SimConstants;
import frc.robot.util.PhoenixSimulationUtil;
import frc.robot.util.PhoenixSimulationUtil.TalonFXMotorControllerSim;
import frc.robot.util.PhoenixSimulationUtil.TalonFXMotorControllerWithRemoteCancoderSim;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  /**
   * Regulates the {@link SwerveModuleConstants} for a single module.Applies specific adjustments to
   * the {@link SwerveModuleConstants} for simulationpurposes.
   */
  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      regulateModuleConstantForSimulation(
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              moduleConstants) {
    // Apply simulation-specific adjustments to module constants
    return moduleConstants
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withDriveMotorGains(
            Slot0Configs.from(SimConstants.simDriveControl.talonFXConfigs().getFirst()));
  }

  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      SwerveModuleSimulation simulation) {
    super(regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    simulation.useDriveMotorController(new TalonFXMotorControllerSim(driveTalon));

    simulation.useSteerMotorController(
        new TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update odometry inputs
    inputs.odometryTimestamps = PhoenixSimulationUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();

    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }
}
