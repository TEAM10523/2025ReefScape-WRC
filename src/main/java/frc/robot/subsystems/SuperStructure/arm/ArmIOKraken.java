package frc.robot.subsystems.SuperStructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOKraken implements ArmIO {
  private final TalonFX armMotor;

  // Status Signals
  private final StatusSignal<Angle> armPosition;
  private final StatusSignal<AngularVelocity> armVelocity;
  private final StatusSignal<Voltage> armAppliedVolts;
  private final StatusSignal<Current> armSupplyCurrent;
  private final StatusSignal<Current> armTorqueCurrent;
  private final StatusSignal<AngularAcceleration> armacceleration;
  private final StatusSignal<Temperature> armTempCelsius;

  private double armRadius = ArmConstants.armRadius;

  private final TalonFXConfiguration armFXConfiguration = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC motionMagicFOC = new MotionMagicTorqueCurrentFOC(0.0);
  private final NeutralOut neutralControl = new NeutralOut();

  public ArmIOKraken() {
    armMotor = new TalonFX(ArmConstants.id);
    armFXConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.armGearRatio;

    armFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /*Torque Current Limiting */
    armFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    armFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    // armFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    // armFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    // armFXConfiguration.CurrentLimits.StatorCurrentLimit = 120.0;

    /* Current Limiting */
    armFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    armFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
    armFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 60;
    armFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.01;

    /* Open and Closed Loop Ramping */
    armFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    armFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

    armMotor.getConfigurator().apply(armFXConfiguration);
    resetPosition(ArmConstants.armInitialAngle);

    // Set signals
    armPosition = armMotor.getPosition();
    armVelocity = armMotor.getVelocity();
    armAppliedVolts = armMotor.getMotorVoltage();
    armSupplyCurrent = armMotor.getSupplyCurrent();
    armTorqueCurrent = armMotor.getTorqueCurrent();
    armacceleration = armMotor.getAcceleration();
    armTempCelsius = armMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        armPosition,
        armVelocity,
        armAppliedVolts,
        armSupplyCurrent,
        armTorqueCurrent,
        armacceleration,
        armTempCelsius);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armMotorConnected =
        BaseStatusSignal.refreshAll(
                armPosition,
                armVelocity,
                armAppliedVolts,
                armSupplyCurrent,
                armTorqueCurrent,
                armacceleration,
                armTempCelsius)
            .isOK();
    inputs.armPositionRads = armPosition.getValueAsDouble() * 2 * Math.PI;
    inputs.armOmegaRadPerSec = armVelocity.getValueAsDouble() * 2 * Math.PI;
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armSupplyCurrentAmps = armSupplyCurrent.getValueAsDouble();
    inputs.armTorqueCurrentAmps = armTorqueCurrent.getValueAsDouble();
    inputs.armTempCelsius = armTempCelsius.getValueAsDouble();
    inputs.armAlphaRadsPerSecSquared = armacceleration.getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void runVolts(double volts) {
    armMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runPositionSetpoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    armMotor.setControl(
        positionControl
            .withPosition(angleRads / (2 * Math.PI))
            .withVelocity(omegaRadPerSec / (2 * Math.PI))
            .withFeedForward(torque * ArmConstants.Kf));
  }

  @Override
  public void runMotionMagicPosition(double angleRads, double omegaRadsPerSec, double torque) {
    motionMagicFOC.Position = angleRads / (2 * Math.PI);
    armMotor.setControl(
        motionMagicFOC.withFeedForward(
            torque * ArmConstants.Kf + omegaRadsPerSec * ArmConstants.Kv));
  }

  @Override
  public void setPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {
    armFXConfiguration.Slot0.kP = kP;
    armFXConfiguration.Slot0.kI = kI;
    armFXConfiguration.Slot0.kD = kD;
    armFXConfiguration.Slot0.kS = kS;
    armFXConfiguration.Slot0.kV = kV;
    armFXConfiguration.Slot0.kA = kA;
    armFXConfiguration.MotionMagic.MotionMagicAcceleration = MotionMagicAcceleration;
    armFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity;
    armMotor.getConfigurator().apply(armFXConfiguration, 0.01);
  }

  @Override
  public void resetPosition(double angleRads) {
    angleRads = angleRads / (2 * Math.PI);
    armMotor.getConfigurator().setPosition(angleRads);
  }

  @Override
  public void stop() {
    armMotor.setControl(neutralControl);
  }
}
