package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSim implements IntakeIO {
  private final DCMotor simMotorModel = DCMotor.getNeo550(1);
  private final DCMotorSim simMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(simMotorModel, 0.0001, 1.0), simMotorModel);
  
  private double appliedVolts = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (isStopped) {
      appliedVolts = 0;
      simMotor.setInputVoltage(0);
    } else {
      simMotor.setInputVoltage(appliedVolts);
    }

    simMotor.update(kSimLoopPeriodSecs);

    inputs.motorConnected = true;
    inputs.appliedVolts = isStopped ? 0 : appliedVolts;
    inputs.supplyCurrentAmps = simMotor.getCurrentDrawAmps();
  } // End updateInputs

  @Override
  public void setVoltage(double volts) {
    appliedVolts = kMotorInverted ? -volts : volts;
    isStopped = false;
  } // End setVoltage

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
