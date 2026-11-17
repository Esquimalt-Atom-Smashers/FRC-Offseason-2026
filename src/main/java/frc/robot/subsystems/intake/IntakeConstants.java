package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig;

public final class IntakeConstants {
  /* CAN ID of the intake motor */
  public static final int kMotorID = 0; // TODO: GET ID

  /* Idle behaviour when motor power is 0 */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /* If true, positive voltage will spin the intake in the opposite direction */
  public static final boolean kMotorInverted = false;

  /* Smart current limit. */
  public static final int kSmartCurrentLimitAmps = 60;

  /* Open-loop ramp rate from 0 to full input. Limits current spikes on step changes. */
  public static final double kOpenLoopRampRate = 0.3;

  /* Periods for sending signals to the motor */
  public static final int kSignalsPeriodMs = 127;
  public static final int kEncoderVelocitySignalPeriodMs = 271;
  
  /* The time between each sim update. */
  public static final double kSimLoopPeriodSecs = 0.02;

  /* The maximum voltage magnitude of the Intake motor */
  public static final double kMaxVoltage = 8.0;
  /* The minimum voltage magnitude of the Intake motor */
  public static final double kMinVoltage = -kMaxVoltage;

  /* Voltage in idle state. */
  public static final double kIdleVoltage = 0.0;
  /* Voltage in intaking state. */
  public static final double kIntakingVoltage = 0.0; // TODO: find prefered intake voltage
  /* Voltage in outtaking state. */
  public static final double kOuttakingVoltage = -kIntakingVoltage;
}
