package frc.robot.subsystems.SuperStructure.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean armMotorConnected = true;

    public double armPositionRads = 0.0;
    public double armOmegaRadPerSec = 0.0;
    public double armAlphaRadsPerSecSquared = 0.0;
    public double armAppliedVolts = 0.0;
    public double armSupplyCurrentAmps = 0.0;
    public double armTorqueCurrentAmps = 0.0;
    public double armTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Run arm motor at volts */
  default void runVolts(double volts) {}

  /** Run to arm positionsetpoint with feedforward */
  default void runPositionSetpoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {}

  /** Run to arm positionsetpoint with motionmagic */
  default void runMotionMagicPosition(double angleRads, double omegaRadsPerSec, double torque) {}

  default void setPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {}

  /** resetPosition */
  default void resetPosition(double angleRads) {}

  /** Disable output to all motors */
  default void stop() {}
}
