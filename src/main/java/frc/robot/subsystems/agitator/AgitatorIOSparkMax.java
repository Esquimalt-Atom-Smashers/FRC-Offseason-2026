package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.kEncoderVelocitySignalPeriodMs;
import static frc.robot.subsystems.agitator.AgitatorConstants.kIdleMode;
import static frc.robot.subsystems.agitator.AgitatorConstants.kMaxVoltage;
import static frc.robot.subsystems.agitator.AgitatorConstants.kMotorId;
import static frc.robot.subsystems.agitator.AgitatorConstants.kMotorInverted;
import static frc.robot.subsystems.agitator.AgitatorConstants.kOpenLoopRampRateSec;
import static frc.robot.subsystems.agitator.AgitatorConstants.kSignalsPeriodMs;
import static frc.robot.subsystems.agitator.AgitatorConstants.kSmartCurrentLimitAmps;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class AgitatorIOSparkMax implements AgitatorIO{
    
    private final SparkMax kMotor;
    
    public AgitatorIOSparkMax() {
        kMotor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);

        var sparkMaxConfig = new SparkMaxConfig();
        
        sparkMaxConfig.idleMode(kIdleMode);
        sparkMaxConfig.inverted(kMotorInverted);
        sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
        sparkMaxConfig.openLoopRampRate(kOpenLoopRampRateSec);
        sparkMaxConfig.voltageCompensation(Constants.kNominalVoltage);
        sparkMaxConfig.signals
            .appliedOutputPeriodMs(kSignalsPeriodMs)
            .busVoltagePeriodMs(kSignalsPeriodMs)
            .outputCurrentPeriodMs(kSignalsPeriodMs)
            .primaryEncoderPositionPeriodMs(kEncoderVelocitySignalPeriodMs)
            .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
        kMotor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        kMotor.setPeriodicFrameTimeout(0);
    }

    @Override
    public void updateInputs(AgitatorIOInputs inputs) {
        inputs.motorConnected = kMotor.getLastError() == REVLibError.kOk;
        inputs.appliedVolts = kMotor.getAppliedOutput();
        inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double volts, boolean ignoreLimits) {
        double clampedVoltage = ignoreLimits 
            ? MathUtil.clamp(volts, -Constants.kNominalVoltage, Constants.kNominalVoltage)
            : MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
        kMotor.setVoltage(clampedVoltage);
    }

    @Override
    public void stop() {
        kMotor.set(0.0);
    }
}
