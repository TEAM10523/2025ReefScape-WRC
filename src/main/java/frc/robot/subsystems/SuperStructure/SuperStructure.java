package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.arm.Arm;
import frc.robot.subsystems.SuperStructure.arm.ArmConstants;
import frc.robot.subsystems.SuperStructure.elevator.Elevator;
import frc.robot.subsystems.SuperStructure.elevator.ElevatorConstants;
import frc.robot.subsystems.SuperStructure.intake.Intake;
import frc.robot.subsystems.SuperStructure.intake.IntakeConstants;
import frc.robot.subsystems.SuperStructure.wrist.Wrist;
import frc.robot.util.BlackholePlanner.Setpoint2d;
import frc.robot.util.BlackholePlanner.Trajectory2d;
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperStructure extends SubsystemBase {
  private final Arm arm;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Intake intake;

  private upperFeedFoward upperfeedfoward;
  // load trajectories
  Trajectory2d Rest2Gnd = new Trajectory2d("Rest2GndIntake", 1);

  Trajectory2d Rest2L1 = new Trajectory2d("Rest2L1", 1);
  Trajectory2d Rest2L2 = new Trajectory2d("Rest2L2", 1);
  Trajectory2d Rest2L3 = new Trajectory2d("Rest2L3", 1);
  Trajectory2d Rest2L4 = new Trajectory2d("Rest2L4", 1);

  Trajectory2d ScoreL2 = new Trajectory2d("ScoreL2", 1);
  Trajectory2d ScoreL3 = new Trajectory2d("ScoreL3", 1);
  Trajectory2d ScoreL4 = new Trajectory2d("ScoreL4", 1);

  Trajectory2d Rest2RemoveL2 = new Trajectory2d("Rest2RemoveL2", 1);
  Trajectory2d Rest2RemoveL3 = new Trajectory2d("Rest2RemoveL3", 1);

  Trajectory2d RemoveL2 = new Trajectory2d("RemoveL2", 1);
  Trajectory2d RemoveL3 = new Trajectory2d("RemoveL3", 1);
  @AutoLogOutput double[] torque;

  public SuperStructure(Arm arm, Elevator elevator, Wrist wrist, Intake intake) {
    this.arm = arm;
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;
    this.upperfeedfoward =
        new upperFeedFoward(
            ArmConstants._armConfig,
            ElevatorConstants._elevatorConfig,
            IntakeConstants._intakeConfig);
  }

  public void SetSetpoint2d(Setpoint2d _Setpoint2d, double wristAngle) {
    torque =
        upperfeedfoward.toMotorTorque(
            arm.getAngleRads(),
            elevator.getPositionMeters(),
            _Setpoint2d.a0,
            arm.getOmegaRadPerSec(),
            _Setpoint2d.a1);
    // torques[0]就是armtorque
    // torques[1]就是liftorque
    elevator.runSetPoint(_Setpoint2d.x0, _Setpoint2d.v0, _Setpoint2d.a0, torque[0]);
    arm.runSetPoint(_Setpoint2d.x1, _Setpoint2d.v1, _Setpoint2d.a1, torque[1]);
    wrist.runMotionMagicPosition(wristAngle);
  }

  public void SetMotionMagic(double elevatorHeight, double armAngle, double wristAngle) {
    torque =
        upperfeedfoward.toMotorTorque(
            arm.getAngleRads(), elevator.getPositionMeters(), 0, arm.getOmegaRadPerSec(), 0);
    elevator.runMotionMagicPosition(elevatorHeight, 0, torque[0]);
    arm.runMotionMagicPosition(armAngle, 0, torque[1]);
    wrist.runMotionMagicPosition(wristAngle);
  }

  public boolean atGoal() {
    return elevator.atGoal() && arm.atGoal() && wrist.atGoal();
  }
}
