package frc.robot.subsystems.agitator;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import static frc.robot.subsystems.agitator.AgitatorConstants.*;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;


import frc.robot.util.TelemetryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;


 public class Agitator extends SubsystemBase{
  private static final String kTargetVoltageKey = "Agitator/TargetVoltage";
  private static final double kTargetVoltageWidgetEpsilon = 1e-3;

  // All possible states: Shooting, Reversing, Idle, or Manual
  public enum State {
    SHOOTING,
    REVERSING,
    IDLE,
    MANUAL
  } // End State

  private final AgitatorIO agitatorIO;
  private final AgitatorIO.AgitatorIOInputs agitatorInputs = new AgitatorIO.AgitatorIOInputs();
  private final String logRoot;

  //The current state
  private State state = State.IDLE;
  private double targetVoltage = kIdleVoltage;
  private BooleanSupplier ignoreLimitSupplier = () -> false;
  private double lastTargetVoltageDashboardWrite = Double.NaN;
  
  public Agitator(AgitatorIO io) {
    this(io, "");
  } 

  public Agitator(AgitatorIO io, String logRoot){
    agitatorIO = io;
    this.logRoot = logRoot;
  }

  public void periodic() {
  agitatorIO.updateInputs(agitatorInputs);
  
  Logger.recordOutput(logRoot + "Subsystems/Agitator/Inputs/MotorConnected", agitatorInputs.motorConnected);
  Logger.recordOutput(logRoot + "Subsystems/Agitator/Inputs/AppliedVolts", TelemetryUtil.roundToTwoDecimals(agitatorInputs.appliedVolts));
  Logger.recordOutput(logRoot + "Subsystems/Agitator/Inputs/SupplyCurrentAmps", TelemetryUtil.roundToTwoDecimals(agitatorInputs.supplyCurrentAmps));
  Logger.recordOutput(logRoot + "Subsystems/Agitator/TargetVolts", TelemetryUtil.roundToTwoDecimals(getTargetVoltage()));
  Logger.recordOutput(logRoot + "Subsystems/Agitator/State", state.name());
 
  if (DriverStation.isDisabled()) {
    agitatorIO.stop();
    lastTargetVoltageDashboardWrite = Double.NaN;
    return;
  }

  boolean useSmartDashboardTarget = ignoreLimitSupplier.getAsBoolean();
  
  if (useSmartDashboardTarget) {
    double dashboardVolts = SmartDashboard.getNumber(kTargetVoltageKey, targetVoltage);
    boolean dashboardTargetEdited = 
      !Double.isNaN(lastTargetVoltageDashboardWrite)
        && Math.abs(dashboardVolts - lastTargetVoltageDashboardWrite) > kTargetVoltageWidgetEpsilon;
    if (dashboardTargetEdited) {
      setTargetVoltage(dashboardVolts);
      if(state == State.IDLE) {
        state = State.MANUAL;
      }
    }
  }
  targetVoltage = TelemetryUtil.roundToTwoDecimals(targetVoltage);
  SmartDashboard.putNumber(kTargetVoltageKey, targetVoltage);
  lastTargetVoltageDashboardWrite = targetVoltage;

  switch (state) {

    case IDLE:
      agitatorIO.stop();
      break;
    
    case SHOOTING:
    case REVERSING: 
    case MANUAL:
      agitatorIO.setVoltage(targetVoltage, ignoreLimitSupplier.getAsBoolean());
      break;
    
    default: 
      agitatorIO.stop();
      break;
  }
  } // End Periodic

  public void setShootingState() {
    state = State.SHOOTING;
    setTargetVoltage(kShootingVoltage);
  }

  public void setReversingState() {
    state = State.REVERSING;
    setTargetVoltage(kReverseVoltage);
  }

  public void setIdleState() {
    state = State.IDLE;
    setTargetVoltage(kIdleVoltage);
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

  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitSupplier = supplier != null ? supplier : () -> false;
  }

  public void stepVoltage(double stepVoltage) {
    boolean wasIdle = getState() == State.IDLE;
    state = State.MANUAL;
    boolean ignoreLimits = ignoreLimitSupplier.getAsBoolean();

    if (wasIdle) {
      setTargetVoltage(ignoreLimits 
      ? MathUtil.clamp(stepVoltage, -Constants.kNominalVoltage, Constants.kNominalVoltage)
      : MathUtil.clamp(stepVoltage, -kMaxVoltage, kMaxVoltage));
    }
    else {
      double stepTargetVoltage = getTargetVoltage() + stepVoltage;
      setTargetVoltage(ignoreLimits 
      ? MathUtil.clamp(stepTargetVoltage, -Constants.kNominalVoltage, Constants.kNominalVoltage)
      : MathUtil.clamp(stepTargetVoltage, -kMaxVoltage, kMaxVoltage));
    }

    if (Math.abs(getTargetVoltage()) < 1e-6) {
      setIdleState();
    }
  }
}

