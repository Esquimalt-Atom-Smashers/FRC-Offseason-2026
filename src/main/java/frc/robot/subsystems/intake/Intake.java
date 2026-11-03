package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public enum State {
    INTAKING,
    OUTTAKING,
    IDLE,
    MANUAL
  }

  private State state = State.IDLE;
}
