package frc.robot.subsystems.SuperStructure.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean intakeMotorConnected = true;

    public double intakePositionRads = 0.0;
    public double intakeVelocityMetersPerSec = 0.0;
    public double intakeAlpha = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public double intakeTorqueCurrentAmps = 0.0;
    public double intakeTempCelsius = 0.0;
    public boolean intakeDetected = true;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Run intake motor at volts */
  default void runVolts(double volts) {}

  /** Run to intake velocity setpoint with feedforward */
  default void runVelocitySetpoint(double velocityMetersPerSec, double acceleration) {}

  /** Configure intake PID */
  default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  /** Disable output to all motors */
  default void stop() {}
}
