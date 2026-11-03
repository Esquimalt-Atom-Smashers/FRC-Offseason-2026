package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kIdleVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kIntakingVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kOuttakingVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TelemetryUtil;

public class Intake extends SubsystemBase {

  public enum State {
    INTAKING,
    OUTTAKING,
    IDLE,
    MANUAL
  }

  private State state = State.IDLE;
  private double targetVoltage = TelemetryUtil.roundToTwoDecimals(kIdleVoltage);

  private final IntakeIO intakeIO;
  private final IntakeIO.IntakeIOInputs intakeIOInputs = new IntakeIO.IntakeIOInputs();

  public Intake(IntakeIO io) {
    intakeIO = io;
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      intakeIO.stop();
      return;
    }

    switch (state) {
      case IDLE:
        intakeIO.stop();
        break;
      case INTAKING:
      case OUTTAKING:
      case MANUAL:
        intakeIO.setVoltage(targetVoltage);
        break;
      default:
        intakeIO.stop();
        break;
    }
  }

  public void setIdleState() {
    state = State.IDLE;
    setTargetVoltage(kIdleVoltage);
  }

  public void setIntakingState() {
    state = State.INTAKING;
    setTargetVoltage(kIntakingVoltage);
  }

  public void setOuttakingState() {
    state = State.OUTTAKING;
    setTargetVoltage(kOuttakingVoltage);
  }

  public State getState() {
    return state;
  }

  public void setTargetVoltage(double volts) {
    targetVoltage = TelemetryUtil.roundToTwoDecimals(volts);
  }

  public double getTargetVoltage() {
    return targetVoltage;
  }
}
