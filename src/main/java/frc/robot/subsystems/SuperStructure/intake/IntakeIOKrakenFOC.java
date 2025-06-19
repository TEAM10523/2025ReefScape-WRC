package frc.robot.subsystems.SuperStructure.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class IntakeIOKrakenFOC implements IntakeIO{
     //hardwares
    private final TalonFX intakeMotor;
    private final DigitalInput m_IntakeSensor;

    //Status Signals
    private final StatusSignal<Angle> intakePosition;
    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> intakeAppliedVolts;
    private final StatusSignal<Current> intakeSupplyCurrent;
    private final StatusSignal<Current> intakeTorqueCurrent;
    private final StatusSignal<Temperature> intakeTempCelsius;
    private final StatusSignal<AngularAcceleration> intakeacceleration; 

    private double wheelRadius = IntakeConstants.wheelRadius;

    private final TalonFXConfiguration intakeFXConfiguration = new TalonFXConfiguration();

     // Control
    private final TorqueCurrentFOC torqueCurrentControl = new TorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = 
        new VelocityTorqueCurrentFOC(0);
    private final NeutralOut neutralControl = new NeutralOut();

    public IntakeIOKrakenFOC(){
        intakeMotor = new TalonFX(IntakeConstants.id);
        m_IntakeSensor=new DigitalInput(IntakeConstants.intakePhotoSensorChannel);
        intakeFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        intakeFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
        intakeFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -100.0;
        intakeFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

        intakeFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeFXConfiguration.CurrentLimits.StatorCurrentLimit = 200.0;

        intakeFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;
        intakeFXConfiguration.CurrentLimits.SupplyCurrentLimit = 200.0;
        intakeFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 200.0;
        intakeFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.02;

        intakeFXConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        intakeFXConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        intakeFXConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.intakeGearRatio;

        intakeMotor.getConfigurator().apply(intakeFXConfiguration);

         //Set signals
         intakePosition = intakeMotor.getPosition();
         intakeVelocity = intakeMotor.getVelocity();
         intakeAppliedVolts = intakeMotor.getMotorVoltage();
         intakeSupplyCurrent = intakeMotor.getSupplyCurrent();
         intakeTorqueCurrent = intakeMotor.getTorqueCurrent();
         intakeacceleration = intakeMotor.getAcceleration();
         intakeTempCelsius = intakeMotor.getDeviceTemp();
 
         BaseStatusSignal.setUpdateFrequencyForAll(
             100.0, 
             intakePosition,
             intakeVelocity,
             intakeAppliedVolts,
             intakeSupplyCurrent,
             intakeTorqueCurrent,
             intakeacceleration,
             intakeTempCelsius);
    }

    @Override
    public void  updateInputs(IntakeIOInputs inputs){
        inputs.intakeMotorConnected = 
            BaseStatusSignal.refreshAll(
                intakePosition,
                intakeVelocity,
                intakeAppliedVolts,
                intakeSupplyCurrent,
                intakeTorqueCurrent,
                intakeTempCelsius)
            .isOK();
            inputs.intakePositionRads = intakePosition.getValueAsDouble()*2*Math.PI;
            inputs.intakeVelocityMetersPerSec = intakeVelocity.getValueAsDouble()*2*Math.PI*wheelRadius;
            inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
            inputs.intakeSupplyCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
            inputs.intakeTorqueCurrentAmps = intakeTorqueCurrent.getValueAsDouble();
            inputs.intakeTempCelsius = intakeTempCelsius.getValueAsDouble();
            inputs.intakeAlpha = intakeacceleration.getValueAsDouble()*2*Math.PI*wheelRadius;
            inputs.intakeDetected=!m_IntakeSensor.get();
    }

    @Override
    public void runVolts(double volts) {
        double targetCurrentAmps = volts;
        intakeMotor.setControl(torqueCurrentControl.withOutput(targetCurrentAmps));
        SmartDashboard.putNumber("Target Torque Current (A)", targetCurrentAmps);
        
    }

    @Override
    public void runVelocitySetpoint(double velocityMetersPerSec, double acceleration) {
        SmartDashboard.putNumber("run", velocityMetersPerSec);
        intakeMotor.setControl(velocityTorqueCurrentFOC
            .withVelocity(velocityMetersPerSec / ((2 * Math.PI) * wheelRadius))
            .withAcceleration(acceleration / (2 * Math.PI)));
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
        intakeFXConfiguration.Slot0.kP = kP;
        intakeFXConfiguration.Slot0.kI = kI;
        intakeFXConfiguration.Slot0.kD = kD;
        intakeFXConfiguration.Slot0.kS = kS;
        intakeFXConfiguration.Slot0.kV = kV;
        intakeFXConfiguration.Slot0.kA = kA;
        intakeMotor.getConfigurator().apply(intakeFXConfiguration, 0.01);
    }

    @Override
    public void stop() {
        intakeMotor.setControl(neutralControl);
    }

}
