// Thanks team 5000 for the naming "inspiration"

package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ModeConstants.Mode;
import frc.robot.util.ControlConstantsBuilder.ControlConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableControls {
  private static final List<Entry> entries = new ArrayList<>();

  private static class Entry {
    public final Runnable callback;
    public final TunableNumber[] numbers;

    public Entry(Runnable callback, TunableNumber[] numbers) {
      this.callback = callback;
      this.numbers = numbers;
    }
  }

  public static class TunableNumber {
    private final LoggedNetworkNumber number;
    private boolean hasLastValue = false;
    private double lastValue = 0;

    public TunableNumber(String key, double defaultValue) {
      this.number = new LoggedNetworkNumber(key, defaultValue);
    }

    public double get() {
      return number.getAsDouble();
    }

    public boolean hasChanged() {
      double current = get();

      if (!hasLastValue) {
        hasLastValue = true;
        lastValue = current;
        return true;
      }

      if (current != lastValue) {
        lastValue = current;
        return true;
      }

      return false;
    }
  }

  public static void register(Runnable callback, TunableNumber... numbers) {
    Objects.requireNonNull(callback);
    Objects.requireNonNull(numbers);

    entries.add(new Entry(callback, numbers));
  }

  public static void periodic() {
    for (Entry entry : entries) {
      for (TunableNumber number : entry.numbers) {
        if (number.hasChanged() && ModeConstants.robotMode != Mode.REAL_COMPETITION) entry.callback.run();
      }
    }
  }

  public static void registerTalonFXSlotTuning(
      TalonFX motor, int slot, String name, ControlConstantsBuilder defaults) {
    Objects.requireNonNull(motor);
    Objects.requireNonNull(name);
    Objects.requireNonNull(defaults);

    ControlConstants defaultConstants = defaults.in(Radians, Seconds);

    TunableNumber kP = new TunableNumber("/Tuning/" + name + "/kP", defaultConstants.kP());
    TunableNumber kI = new TunableNumber("/Tuning/" + name + "/kI", defaultConstants.kI());
    TunableNumber kD = new TunableNumber("/Tuning/" + name + "/kD", defaultConstants.kD());
    TunableNumber kS = new TunableNumber("/Tuning/" + name + "/kS", defaultConstants.kS());
    TunableNumber kV = new TunableNumber("/Tuning/" + name + "/kV", defaultConstants.kV());
    TunableNumber kA = new TunableNumber("/Tuning/" + name + "/kA", defaultConstants.kA());

    Runnable apply =
        () -> {
          ControlConstantsBuilder builder =
              ControlConstantsBuilder.fromRadiansAndSeconds()
                  .pid(kP.get(), kI.get(), kD.get())
                  .sva(kS.get(), kV.get(), kA.get());

          Pair<SlotConfigs, ?> talonConfigs = builder.talonFXConfigs();
          SlotConfigs slotConfigs = talonConfigs.getFirst();

          switch (slot) {
            case 0 -> motor.getConfigurator().apply(Slot0Configs.from(slotConfigs));
            case 1 -> motor.getConfigurator().apply(Slot1Configs.from(slotConfigs));
            case 2 -> motor.getConfigurator().apply(Slot2Configs.from(slotConfigs));
            default -> throw new IllegalArgumentException("Unsupported slot: " + slot);
          }
        };

    register(apply, kP, kI, kD, kS, kV, kA);
  }

  public static void registerSparkMaxClosedLoopTuning(
      SparkMax motor, String name, ControlConstantsBuilder defaults) {
    Objects.requireNonNull(motor);
    Objects.requireNonNull(name);
    Objects.requireNonNull(defaults);

    ControlConstants defaultConstants = defaults.in(Radians, Seconds);

    TunableNumber kP = new TunableNumber("/Tuning/" + name + "/kP", defaultConstants.kP());
    TunableNumber kI = new TunableNumber("/Tuning/" + name + "/kI", defaultConstants.kI());
    TunableNumber kD = new TunableNumber("/Tuning/" + name + "/kD", defaultConstants.kD());
    TunableNumber kS = new TunableNumber("/Tuning/" + name + "/kS", defaultConstants.kS());
    TunableNumber kV = new TunableNumber("/Tuning/" + name + "/kV", defaultConstants.kV());
    TunableNumber kA = new TunableNumber("/Tuning/" + name + "/kA", defaultConstants.kA());

    Runnable apply =
        () -> {
          ControlConstantsBuilder builder =
              ControlConstantsBuilder.fromRadiansAndSeconds()
                  .pid(kP.get(), kI.get(), kD.get())
                  .sva(kS.get(), kV.get(), kA.get());

          SparkMaxConfig config = new SparkMaxConfig();
          config.apply(builder.revClosedLoopConfig());

          motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        };

    register(apply, kP, kI, kD, kS, kV, kA);
  }

  public static void registerPIDControllerTuning(PIDController controller, String name) {
    TunableNumber kP = new TunableNumber("/Tuning/" + name + "/kP", controller.getP());
    TunableNumber kI = new TunableNumber("/Tuning/" + name + "/kI", controller.getI());
    TunableNumber kD = new TunableNumber("/Tuning/" + name + "/kD", controller.getD());

    Runnable apply =
        () -> {
          controller.setP(kP.get());
          controller.setI(kI.get());
          controller.setD(kD.get());
        };

    register(apply, kP, kI, kD);
  }

  public static void registerProfiledPIDControllerTuning(
      ProfiledPIDController controller, String name) {
    TunableNumber kP = new TunableNumber("/Tuning/" + name + "/kP", controller.getP());
    TunableNumber kI = new TunableNumber("/Tuning/" + name + "/kI", controller.getI());
    TunableNumber kD = new TunableNumber("/Tuning/" + name + "/kD", controller.getD());

    Runnable apply =
        () -> {
          controller.setP(kP.get());
          controller.setI(kI.get());
          controller.setD(kD.get());
        };

    register(apply, kP, kI, kD);
  }
}
