package frc.robot.subsystems.SuperStructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOKrakenFOC implements ElevatorIO {
  // hardwares
  private final TalonFX elevatorMotor;

  // Status Signals
  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> elevatorAppliedVolts;
  private final StatusSignal<Current> elevatorSupplyCurrent;
  private final StatusSignal<Current> elevatorTorqueCurrent;
  private final StatusSignal<AngularAcceleration> elevatorAcceleration;
  private final StatusSignal<Temperature> elevatorTempCelsius;

  private double elevatorRadius = ElevatorConstants.sprocketRadius;

  private final TalonFXConfiguration elevatorFXConfiguration = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC motionMagicFOC = new MotionMagicTorqueCurrentFOC(0.0);
  private final NeutralOut neutralControl = new NeutralOut();

  public ElevatorIOKrakenFOC() {
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorId);
    elevatorFXConfiguration.Feedback.SensorToMechanismRatio = ElevatorConstants.elevatorGearRatio;

    elevatorFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /*Torque Current Limiting */
    elevatorFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 200.0;
    elevatorFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -200.0;
    // elevatorFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    // elevatorFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    // elevatorFXConfiguration.CurrentLimits.StatorCurrentLimit = 120.0;

    /* Current Limiting */
    elevatorFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;
    elevatorFXConfiguration.CurrentLimits.SupplyCurrentLimit = 200;
    elevatorFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 100;
    elevatorFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.01;

    /* Open and Closed Loop Ramping */
    elevatorFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    elevatorFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

    elevatorMotor.getConfigurator().apply(elevatorFXConfiguration);
    resetPosition(ElevatorConstants.elevatorInitialPositin / (2 * Math.PI * elevatorRadius));

    // Set signals
    elevatorPosition = elevatorMotor.getPosition();
    elevatorVelocity = elevatorMotor.getVelocity();
    elevatorAppliedVolts = elevatorMotor.getMotorVoltage();
    elevatorSupplyCurrent = elevatorMotor.getSupplyCurrent();
    elevatorTorqueCurrent = elevatorMotor.getTorqueCurrent();
    elevatorAcceleration = elevatorMotor.getAcceleration();
    elevatorTempCelsius = elevatorMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        elevatorPosition,
        elevatorVelocity,
        elevatorAppliedVolts,
        elevatorSupplyCurrent,
        elevatorTorqueCurrent,
        elevatorAcceleration,
        elevatorTempCelsius);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorMotorConnected =
        BaseStatusSignal.refreshAll(
                elevatorPosition,
                elevatorVelocity,
                elevatorAppliedVolts,
                elevatorSupplyCurrent,
                elevatorTorqueCurrent,
                elevatorAcceleration,
                elevatorTempCelsius)
            .isOK();
    inputs.elevatorPositionMeters =
        elevatorPosition.getValueAsDouble() * 2 * Math.PI * elevatorRadius;
    inputs.elevatorVelocityMetersPerSec =
        elevatorVelocity.getValueAsDouble() * 2 * Math.PI * elevatorRadius;
    inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValueAsDouble();
    inputs.elevatorSupplyCurrentAmps = elevatorSupplyCurrent.getValueAsDouble();
    inputs.elevatorTorqueCurrentAmps = elevatorTorqueCurrent.getValueAsDouble();
    inputs.elevatorTempCelsius = elevatorTempCelsius.getValueAsDouble();
    inputs.elevatorAcceleration =
        elevatorAcceleration.getValueAsDouble() * 2 * Math.PI * elevatorRadius;
  }

  @Override
  public void runVolts(double volts) {
    elevatorMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runPositionSetpoint(
      double positionMeters, double velocityMetersPerSec, double acceleration, double torque) {
    elevatorMotor.setControl(
        positionControl
            .withPosition(positionMeters / (2 * Math.PI * elevatorRadius))
            .withVelocity(velocityMetersPerSec / (2 * Math.PI * elevatorRadius))
            .withFeedForward(torque * ElevatorConstants.Kf));
  }

  @Override
  public void runMotionMagicPosition(
      double positionMeters, double velocityMetersPerSec, double torque) {
    // motionMagicFOC.Position = positionMeters / (2 * Math.PI * elevatorRadius);
    elevatorMotor.setControl(
        motionMagicFOC
            .withPosition(positionMeters / (2 * Math.PI * elevatorRadius))
            .withFeedForward(torque * ElevatorConstants.Kf));
  }

  @Override
  public void setPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double KG,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {
    elevatorFXConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorFXConfiguration.Slot0.kP = kP;
    elevatorFXConfiguration.Slot0.kI = kI;
    elevatorFXConfiguration.Slot0.kD = kD;
    elevatorFXConfiguration.Slot0.kS = kS;
    elevatorFXConfiguration.Slot0.kV = kV;
    elevatorFXConfiguration.Slot0.kA = kA;
    elevatorFXConfiguration.Slot0.kG = KG;
    elevatorFXConfiguration.MotionMagic.MotionMagicAcceleration =
        MotionMagicAcceleration / (2 * Math.PI * elevatorRadius);
    elevatorFXConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        MotionMagicCruiseVelocity / (2 * Math.PI * elevatorRadius);
    elevatorMotor.getConfigurator().apply(elevatorFXConfiguration, 0.01);
  }

  @Override
  public void resetPosition(double positionMeters) {
    positionMeters = positionMeters / (2 * Math.PI);
    elevatorMotor.getConfigurator().setPosition(positionMeters);
  }

  @Override
  public void stop() {
    elevatorMotor.setControl(neutralControl);
  }
}
