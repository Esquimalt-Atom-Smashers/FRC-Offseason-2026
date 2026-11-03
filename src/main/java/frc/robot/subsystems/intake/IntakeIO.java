package frc.robot.subsystems.intake;

public interface IntakeIO {

  class IntakeIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {}
}
