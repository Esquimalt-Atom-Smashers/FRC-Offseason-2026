package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kIdleVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kIntakingVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kOuttakingVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TelemetryUtil;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  /* Intake states: Idle, Intaking (pull in), Outtaking (spit out), or Manual. */
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
  private final String logRoot;
  private final String logSubsystem = "Subsystems/Intake/";

  public Intake(IntakeIO io) {
    this(io, "");
  } // End Intake Constructor

  public Intake(IntakeIO io, String logRoot) {
    intakeIO = io;
    this.logRoot = logRoot;
  } // End Intake Constructor 

  public void periodic() {
    intakeIO.updateInputs(intakeIOInputs);
    recordOutputs();

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
  } // End periodic

  /* Records the inputs and values to smart dashboard */
  public void recordOutputs() {
    Logger.recordOutput(logRoot + logSubsystem + "Inputs/MotorConnected", intakeIOInputs.motorConnected);
    Logger.recordOutput(logRoot + logSubsystem + "Inputs/AppliedVolts", intakeIOInputs.appliedVolts);
    Logger.recordOutput(logRoot + logSubsystem + "Inputs/SupplyCurrentAmps", intakeIOInputs.supplyCurrentAmps);
    Logger.recordOutput(logRoot + logSubsystem + "TargetVolts", targetVoltage);
    Logger.recordOutput(logRoot + logSubsystem + "State", state.name());
  } // End recordOutputs

  /* Set the state to idle (motor stopped) */
  public void setIdleState() {
    state = State.IDLE;
    setTargetVoltage(kIdleVoltage);
  } // end setIdleState

  /* Set the state to intaking (suck fuel in) */
  public void setIntakingState() {
    state = State.INTAKING;
    setTargetVoltage(kIntakingVoltage);
  } // End setIntakingState

  /* Set the state to outtaking (spit fuel out) */
  public void setOuttakingState() {
    state = State.OUTTAKING;
    setTargetVoltage(kOuttakingVoltage);
  } // End setOuttakingState

  /* Returns the current state */
  public State getState() {
    return state;
  } // End getState

  /* Sets the target voltage of the motor (Rounded to 2 decimals) */
  public void setTargetVoltage(double volts) {
    targetVoltage = TelemetryUtil.roundToTwoDecimals(volts);
  } // End setTargetVoltage

  /* Returns the target voltage of the motor */
  public double getTargetVoltage() {
    return targetVoltage;
  } // End getTargetVoltage
}
