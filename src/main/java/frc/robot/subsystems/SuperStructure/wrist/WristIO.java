package frc.robot.subsystems.SuperStructure.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
    class WristIOInputs {
        public boolean wristMotorConnected = true;
    
        public double wristPositionRads = 0.0;
        public double wristOmegaRadPerSec = 0.0;
        public double wristAlphaRadsPerSecSquared = 0.0;
        public double wristAppliedVolts = 0.0;
        public double wristSupplyCurrentAmps = 0.0;
        public double wristTorqueCurrentAmps = 0.0;
        public double wristTempCelsius = 0.0;
      } 

       /** Updates the set of loggable inputs. */
      default void updateInputs(WristIOInputs inputs) {}
    
      /** Run wrist motor at volts */
      default void runVolts(double volts) {}
    
      /** Run to wrist positionsetpoint with feedforward */
      default void runPositionSetpoint(double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {}

       /** Run to wrist positionsetpoint with motionmagic */
      default void runMotionMagicPosition(double angleRads) {}
      /** Configure wrist PID */
      default void setPID(double kP, double kI, double kD, double kS, double kV, double kA,
        double MotionMagicAcceleration,double MotionMagicCruiseVelocity) {}

       /** resetPosition */
      default void resetPosition(double angleRads) {}
    
      /** Disable output to all motors */
      default void stop() {}
}
