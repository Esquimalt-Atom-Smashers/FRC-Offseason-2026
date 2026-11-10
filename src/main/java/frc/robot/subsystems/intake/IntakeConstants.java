package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig;

public final class IntakeConstants {
  public static final int kMotorID = 0; // TODO: GET ID

  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  public static final boolean kMotorInverted = false;

  public static final int kSmartCurrentLimitAmps = 60;

  public static final double kOpenLoopRampRate = 0.3;

  public static final int kSignalsPeriodMs = 127;
  public static final int kEncoderVelocitySignalPeriodMs = 271;

  public static final double kIdleVoltage = 0.0;
  public static final double kIntakingVoltage = 0.0;
  public static final double kOuttakingVoltage = -kIntakingVoltage;
}
