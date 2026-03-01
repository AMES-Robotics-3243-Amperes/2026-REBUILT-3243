// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class IndexerConstants {
  public static final int spindexerId = 1;
  public static final double spindexerReduction = 4.0;

  public static final AngularVelocity spindexerIndexingSpeed = RPM.of(600);

  public static final int spindexerCurrentLimit = 80;

  public static final ControlConstantsBuilder spindexerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.00002, 0, 0)
          .sva(0.59649, 0.071983, 0.0041446);

  public static final int kickerId = 4;
  public static final double kickerReduction = 5.0 / 4.0;

  public static final AngularVelocity kickerShootingSpeed = RPM.of(1000);

  public static final int kickerCurrentLimit = 90;

  public static final ControlConstantsBuilder kickerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.000002, 0, 0)
          .sva(0.39338, 0.023391, 0.0025287);
}
