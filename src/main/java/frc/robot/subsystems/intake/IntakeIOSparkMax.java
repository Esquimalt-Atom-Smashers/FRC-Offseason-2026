package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax motor;

  public IntakeIOSparkMax() {
    motor = new SparkMax(kMotorID, SparkLowLevel.MotorType.kBrushless);

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig.openLoopRampRate(kOpenLoopRampRate);
    sparkMaxConfig.voltageCompensation(Constants.kNominalVoltage);
    sparkMaxConfig.signals
      .appliedOutputPeriodMs(kSignalsPeriodMs)
      .busVoltagePeriodMs(kSignalsPeriodMs)
      .outputCurrentPeriodMs(kSignalsPeriodMs)
      .primaryEncoderPositionPeriodMs(kEncoderVelocitySignalPeriodMs)
      .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motor.setPeriodicFrameTimeout(0);
    // TODO: setup config
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    motor.set(volts);
  }

  @Override
  public void stop() {
    motor.set(0.0);
  }
}
