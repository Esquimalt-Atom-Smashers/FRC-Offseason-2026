package frc.robot.subsystems.agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {

    @AutoLog
    class AgitatorIOInputs {
        public boolean motorConnected = false;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
    }

    // Update inputs from the hardware
    default void updateInputs(AgitatorIOInputs inputs) {}

    // Set motor output voltage and whether or not to limit it
    default void setVoltage(double volts, boolean ignoreLimits) {}

    // Set motor to stop
    default void stop() {}
} 
