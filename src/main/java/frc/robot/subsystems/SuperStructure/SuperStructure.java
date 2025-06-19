package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.arm.Arm;
import frc.robot.subsystems.SuperStructure.arm.ArmConstants;
import frc.robot.subsystems.SuperStructure.elevator.Elevator;
import frc.robot.subsystems.SuperStructure.elevator.ElevatorConstants;
import frc.robot.subsystems.SuperStructure.intake.Intake;
import frc.robot.subsystems.SuperStructure.intake.IntakeConstants;
import frc.robot.subsystems.SuperStructure.wrist.Wrist;
import frc.robot.util.BlackholePlanner.Trajectory2d;

public class SuperStructure extends SubsystemBase{
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
     public SuperStructure(Arm arm, Elevator elevator, Wrist wrist, Intake intake){
    this.arm = arm;
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;
    this.upperfeedfoward = new upperFeedFoward(ArmConstants._armConfig, ElevatorConstants._elevatorConfig, IntakeConstants._intakeConfig);

  }
  
}
