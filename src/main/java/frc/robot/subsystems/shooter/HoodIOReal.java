package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ShooterConstants;

public class HoodIOReal implements HoodIO {
  protected final SparkMax sparkMax = new SparkMax(ShooterConstants.hoodId, MotorType.kBrushless);

  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  public HoodIOReal() {
    SparkMaxConfig hoodConfig = new SparkMaxConfig();

    hoodConfig.encoder.positionConversionFactor(1.0 / ShooterConstants.hoodGearReduction);
    hoodConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.hoodGearReduction);
    hoodConfig.idleMode(IdleMode.kCoast).inverted(false);

    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .apply(ShooterConstants.hoodControl.revClosedLoopConfig());

    sparkMax.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = sparkMax.getClosedLoopController();
    encoder = sparkMax.getEncoder();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.angle = Rotations.of(encoder.getPosition());
    inputs.angularVelocity = RPM.of(encoder.getVelocity());
    inputs.appliedVoltage = Volts.of(sparkMax.getAppliedOutput() * sparkMax.getBusVoltage());
  }

  @Override
  public void resetPosition(Angle angle) {
    encoder.setPosition(angle.in(Rotations));
  }

  @Override
  public void runOpenLoop(double output) {
    sparkMax.setVoltage(output);
  }

  @Override
  public void setAngle(Angle angle) {
    closedLoopController.setSetpoint(angle.in(Rotations), ControlType.kPosition);
  }
}
