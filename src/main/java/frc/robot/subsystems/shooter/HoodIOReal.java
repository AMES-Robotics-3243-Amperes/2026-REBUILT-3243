package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
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
  private final AbsoluteEncoder absoluteEncoder;

  private boolean hasResetRelativeEncoder = false;

  public HoodIOReal() {
    SparkMaxConfig hoodConfig = new SparkMaxConfig();

    hoodConfig.encoder.positionConversionFactor(
        1.0
            / (ShooterConstants.encoderToHoodReduction
                * ShooterConstants.motorToEncoderHoodReduction));
    hoodConfig.encoder.velocityConversionFactor(
        1.0
            / (ShooterConstants.encoderToHoodReduction
                * ShooterConstants.motorToEncoderHoodReduction));
    hoodConfig.absoluteEncoder.zeroCentered(true);
    hoodConfig.idleMode(IdleMode.kCoast).inverted(true);

    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .apply(ShooterConstants.hoodControl.revClosedLoopConfig());

    sparkMax.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = sparkMax.getClosedLoopController();
    encoder = sparkMax.getEncoder();
    absoluteEncoder = sparkMax.getAbsoluteEncoder();
  }

  private void resyncRelativeEncoder() {
    Angle absoluteEncoderPosition = Rotations.of(absoluteEncoder.getPosition());

    if (!absoluteEncoderPosition.isEquivalent(Radians.of(0))) {
      encoder.setPosition(
          absoluteEncoderPosition
              .div(ShooterConstants.encoderToHoodReduction)
              .plus(ShooterConstants.hoodAbsoluteEncoderZeroedRotation)
              .in(Rotations));
    }
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // the absolute encoder doesn't report accumulated rotation, but instead gives a single number
    // between 0 and 1 (or -0.5 and 0.5 with how we've configured it). this means that when the hood
    // goes far enough up for the encoder to make a whole rotation, there's a really bad
    // discontinuity that can make things go wrong. thus we run the closed loop control off of the
    // 550's internal encoder, but when we first boot up the robot and when the encoder is close
    // enough to its starting rotation we synchronize the values
    double hoodRotationsForOneAbsEncoderRotation = 1.0 / ShooterConstants.encoderToHoodReduction;

    if (!hasResetRelativeEncoder && Math.abs(sparkMax.getAbsoluteEncoder().getPosition()) > 1e-3) {
      resyncRelativeEncoder();
      hasResetRelativeEncoder = true;
    } else if (encoder.getPosition() < hoodRotationsForOneAbsEncoderRotation / 4) {
      resyncRelativeEncoder();
    }

    inputs.angle = Rotations.of(encoder.getPosition());
    inputs.angularVelocity = RPM.of(encoder.getVelocity());
    inputs.appliedVoltage = Volts.of(sparkMax.getAppliedOutput() * sparkMax.getBusVoltage());
  }

  @Override
  public void setAngle(Angle angle) {
    closedLoopController.setSetpoint(angle.in(Rotations), ControlType.kPosition);
  }
}
