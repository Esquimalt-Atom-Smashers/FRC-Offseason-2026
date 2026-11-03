package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax motor;

  public IntakeIOSparkMax() {
    motor = new SparkMax(kMotorID, SparkLowLevel.MotorType.kBrushless);
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
