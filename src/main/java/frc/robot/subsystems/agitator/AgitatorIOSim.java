package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.kMaxVoltage;
import static frc.robot.subsystems.agitator.AgitatorConstants.kMotorInverted;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class AgitatorIOSim implements AgitatorIO{
    
    private static final double kLoopPeriodSecs = 0.02;

    private final DCMotor kMotor = DCMotor.getNeo550(1);
    private final DCMotorSim kSimMotor = 
        new DCMotorSim(LinearSystemId.createDCMotorSystem(kMotor, 0.001, 1.0), kMotor);

    
    private double appliedVolts = 0.0;
    private boolean isStopped = true;

    @Override
    public void updateInputs(AgitatorIOInputs inputs) {
        if (isStopped) {
            appliedVolts = 0.0;
            kSimMotor.setInputVoltage(appliedVolts);
        }
        else {
            kSimMotor.setInputVoltage(appliedVolts);
        }
        kSimMotor.update(kLoopPeriodSecs);

        inputs.motorConnected = true;
        inputs.appliedVolts = isStopped ? 0.0 : appliedVolts;
        inputs.supplyCurrentAmps = kSimMotor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts, boolean ignoreLimits) {
        // This ensures that the voltage applied is something we are able to supply
        double clampedVoltage = ignoreLimits 
            ? MathUtil.clamp(volts, -Constants.kNominalVoltage, Constants.kNominalVoltage)
            : MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
        appliedVolts = kMotorInverted ? -clampedVoltage : clampedVoltage;
        isStopped = false;
    }

    @Override
    public void stop() {
        isStopped = true;
    }
}
