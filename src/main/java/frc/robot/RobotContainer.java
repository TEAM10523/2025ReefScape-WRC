// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SuperStructureConstants.UpperStructureState;
import frc.robot.commands.Auto.Mid103;
import frc.robot.commands.DefaultSuperStructureCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SuperStructureMotionMagicCommand;
import frc.robot.commands.VerticalIntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.arm.Arm;
import frc.robot.subsystems.SuperStructure.arm.ArmIO;
import frc.robot.subsystems.SuperStructure.arm.ArmIOKraken;
import frc.robot.subsystems.SuperStructure.elevator.Elevator;
import frc.robot.subsystems.SuperStructure.elevator.ElevatorIO;
import frc.robot.subsystems.SuperStructure.elevator.ElevatorIOKrakenFOC;
import frc.robot.subsystems.SuperStructure.intake.Intake;
import frc.robot.subsystems.SuperStructure.intake.IntakeIO;
import frc.robot.subsystems.SuperStructure.intake.IntakeIOKrakenFOC;
import frc.robot.subsystems.SuperStructure.wrist.Wrist;
import frc.robot.subsystems.SuperStructure.wrist.WristIO;
import frc.robot.subsystems.SuperStructure.wrist.WristIOKrakenFOC;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Arm m_Arm;
  private final Elevator m_Elevator;
  private final Intake m_Intake;
  private final Wrist m_Wrist;
  public final SuperStructure m_SuperStructure;
  @AutoLogOutput private int m_LRIndex = 1;
  @AutoLogOutput private int m_LevelIndex = 4;
  private final Vision vision;

  // Controller
  public final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision = new Vision(VisionConstants.ios, VisionConstants.configs, drive);
        m_Arm = new Arm(new ArmIOKraken());
        m_Elevator = new Elevator(new ElevatorIOKrakenFOC());
        m_Intake = new Intake(new IntakeIOKrakenFOC());
        m_Wrist = new Wrist(new WristIOKrakenFOC());
        m_SuperStructure = new SuperStructure(m_Arm, m_Elevator, m_Wrist, m_Intake);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision = null;
        m_Arm = new Arm(new ArmIO() {});
        m_Elevator = new Elevator(new ElevatorIO() {});
        m_Intake = new Intake(new IntakeIO() {});
        m_Wrist = new Wrist(new WristIO() {});
        m_SuperStructure = new SuperStructure(m_Arm, m_Elevator, m_Wrist, m_Intake);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = null;
        m_Arm = new Arm(new ArmIO() {});
        m_Elevator = new Elevator(new ElevatorIO() {});
        m_Intake = new Intake(new IntakeIO() {});
        m_Wrist = new Wrist(new WristIO() {});
        m_SuperStructure = new SuperStructure(m_Arm, m_Elevator, m_Wrist, m_Intake);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Mid103", new Mid103(this));
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_SuperStructure.setDefaultCommand(
    //     new InstantCommand(() -> m_SuperStructure.SetMotionMagic(0, 0, 0)));
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));
    m_SuperStructure.setDefaultCommand(new DefaultSuperStructureCommand(this));

    // Lock to 0° when A button is held
    controller
        .cross()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  ++m_LevelIndex;
                  if (m_LevelIndex > 4) m_LevelIndex = 4;
                }));
    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  --m_LevelIndex;
                  if (m_LevelIndex < 1) m_LevelIndex = 1;
                }));
    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  m_LRIndex = 2;
                }));
    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  m_LRIndex = 1;
                }));
    controller
        .square()
        .whileTrue(
            Commands.defer(
                () -> ScoreCommand.getClosestReefCommand(this, m_LRIndex, m_LevelIndex),
                Set.of(m_SuperStructure, drive)));
    controller
        .triangle()
        .whileTrue(new SuperStructureMotionMagicCommand(UpperStructureState.ScoreL4, this));
    // Commands.defer(
    //     () -> {
    //       return new ScoreCommand(this, new Pose2d(), m_LevelIndex, false);
    //     },
    //     Set.of(m_SuperStructure, drive)));
    controller.R1().whileTrue(new VerticalIntakeCommands(this));
    // Reset gyro to 0° when B button is pressed
    controller
        .circle()
        .onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d()), drive).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Mid103(this);
  }
}
