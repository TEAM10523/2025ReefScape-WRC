package frc.robot.subsystems.SuperStructure.wrist;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristIOKrakenFOC implements WristIO {
  // hardwares
  private final TalonFX wristMotor;

  // Status Signals
  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;
  private final StatusSignal<Voltage> wristAppliedVolts;
  private final StatusSignal<Current> wristSupplyCurrent;
  private final StatusSignal<Current> wristTorqueCurrent;
  private final StatusSignal<AngularAcceleration> wristacceleration;
  private final StatusSignal<Temperature> wristTempCelsius;

  private double wristRadius = WristConstants.wristRadius;

  private final TalonFXConfiguration wristFXConfiguration = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC motionMagicFOC = new MotionMagicTorqueCurrentFOC(0.0);
  private final NeutralOut neutralControl = new NeutralOut();

  public WristIOKrakenFOC() {
    wristMotor = new TalonFX(WristConstants.id);
    wristFXConfiguration.Feedback.SensorToMechanismRatio = WristConstants.wristGearRatio;

    wristFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /*Torque Current Limiting */
    // wristFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 100.0;
    // wristFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -100.0;
    // wristFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    // wristFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    // wristFXConfiguration.CurrentLimits.StatorCurrentLimit = 120.0;

    /* Current Limiting */
    wristFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
    wristFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 60;
    wristFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.01;

    /* Open and Closed Loop Ramping */
    wristFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    wristFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

    wristMotor.getConfigurator().apply(wristFXConfiguration);
    resetPosition(WristConstants.wristInitialAngle);

    // Set signals
    wristPosition = wristMotor.getPosition();
    wristVelocity = wristMotor.getVelocity();
    wristAppliedVolts = wristMotor.getMotorVoltage();
    wristSupplyCurrent = wristMotor.getSupplyCurrent();
    wristTorqueCurrent = wristMotor.getTorqueCurrent();
    wristacceleration = wristMotor.getAcceleration();
    wristTempCelsius = wristMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        wristPosition,
        wristVelocity,
        wristAppliedVolts,
        wristSupplyCurrent,
        wristTorqueCurrent,
        wristacceleration,
        wristTempCelsius);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristMotorConnected =
        BaseStatusSignal.refreshAll(
                wristPosition,
                wristVelocity,
                wristAppliedVolts,
                wristSupplyCurrent,
                wristTorqueCurrent,
                wristacceleration,
                wristTempCelsius)
            .isOK();
    inputs.wristPositionRads = wristPosition.getValueAsDouble() * 2 * Math.PI;
    inputs.wristOmegaRadPerSec = wristVelocity.getValueAsDouble() * 2 * Math.PI;
    inputs.wristAppliedVolts = wristAppliedVolts.getValueAsDouble();
    inputs.wristSupplyCurrentAmps = wristSupplyCurrent.getValueAsDouble();
    inputs.wristTorqueCurrentAmps = wristTorqueCurrent.getValueAsDouble();
    inputs.wristTempCelsius = wristTempCelsius.getValueAsDouble();
    inputs.wristAlphaRadsPerSecSquared = wristacceleration.getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void runVolts(double volts) {
    wristMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runPositionSetpoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    wristMotor.setControl(
        positionControl
            .withPosition(angleRads / (2 * Math.PI))
            .withVelocity(omegaRadPerSec / (2 * Math.PI)));
  }

  @Override
  public void runMotionMagicPosition(double angleRads) {
    SmartDashboard.putNumber("wrist running theta", angleRads);
    motionMagicFOC.Position = angleRads / (2 * Math.PI);
    wristMotor.setControl(motionMagicFOC);
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
    wristFXConfiguration.Slot0.kP = kP;
    wristFXConfiguration.Slot0.kI = kI;
    wristFXConfiguration.Slot0.kD = kD;
    wristFXConfiguration.Slot0.kS = kS;
    wristFXConfiguration.Slot0.kV = kV;
    wristFXConfiguration.Slot0.kA = kA;
    wristFXConfiguration.MotionMagic.MotionMagicAcceleration = MotionMagicAcceleration;
    wristFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity;
    wristMotor.getConfigurator().apply(wristFXConfiguration, 0.01);
  }

  @Override
  public void resetPosition(double angleRads) {
    wristMotor.getConfigurator().setPosition(angleRads);
  }

  @Override
  public void stop() {
    wristMotor.setControl(neutralControl);
  }
}
