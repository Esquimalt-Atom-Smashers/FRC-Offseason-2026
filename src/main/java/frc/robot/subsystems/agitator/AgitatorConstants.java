package frc.robot.subsystems.agitator;

import com.revrobotics.spark.config.SparkBaseConfig;


public class AgitatorConstants {
   
   // Determines the morality of the given subsystem (0 - 10) (higher means more evil)
   public static final double kEvilness = 9.80665;

   // Can ID of Agitator Motor
   public static final int kMotorId = 0;

   // Chooses what to do when recieving zero power (coast or brake)
   public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

   // True if positive voltage means motor spins backwards
   public static final boolean kMotorInverted = false;

   // Smart Current Limit in... you'll never guess this but Amperes!
   public static final int kSmartCurrentLimitAmps = 10;

   // Open loop ramp time from 0 to full output. Limits current spikes on step changes
   public static final double kOpenLoopRampRateSec = 0.0;

   // Period for sending signals to the motor
   public static final int kSignalsPeriodMs = 107;
   public static final int kEncoderVelocitySignalPeriodMs = 251;

   // Max voltage magnitude applied to motor
   public static final double kMaxVoltage = 12.0;

   // Voltage in Idle state
   public static final double kIdleVoltage = 0.0;
   
   // Voltage in Reverse state
   public static final double kReverseVoltage = - 1.0;

   // Voltage in Shooting state
   public static final double kShootingVoltage = 1.0;
   
   // Voltage to increment by per step
   public static final double kStepVolts = 0.10;
}
