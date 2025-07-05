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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class AutoAlignConstants {
    public static final double kTranslationP = 0.1;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;
    public static final double kPositionTolerance = 0.03;
    public static final double kAngleTolerance = 0.05;
    public static final double kToleranceDeg = 2.0;
    public static final double kMaxVelocity = 2.5;
    public static final double kMaxAngularvelocity = Math.PI;
    public static final double PlacementThreshold = 1.0;
    public static final double ScoreThresholdDirection = 0.1;
    public static final double ScoreThresholdDistance = 0.03;
    public static final double ScoreThresholdElevator = 0.1;
  }

  public class SuperStructureConstants {
    public enum UpperStructureState {
      ScoreL1(0.05, 0.99, 0),
      ScoreL2(0.281, 2.0, 1.8),
      ScoreL3(0.744, 1.98, 1.8),
      ScoreL4(1.4185, 1.865, 1.8),
      RemoveL3(0.694, 2.909464, 1.8),
      RemoveL2(0.758, 2.734, 1.8),
      GroundIntake(0.05, -0.15, 0),
      StationIntake(0.27, 1.3, 0),
      VerticalIntake(0.03, 0, 1.8),
      Rest(0, 1.57079, 1.8),
      PrepareClimb(0.03, 0, 1.8),
      Climb(0.03, 0, 1.8);

      public final double elevator_height;
      public final double arm_theta;
      public final double wrist_theta;

      UpperStructureState(double elevator_height, double arm_theta, double wrist_theta) {
        this.elevator_height = elevator_height;
        this.arm_theta = arm_theta;
        this.wrist_theta = wrist_theta;
      }
    }

    


  }

  public enum Location {
    Shanghai(9.794),
    Hainan(9.783),
    Huston(9.79),
    California(9.796);

    public final double gravity;

    Location(double gravity) {
      this.gravity = gravity;
    }
  }

  public static final Location location = Location.Shanghai;

  public static final double g = location.gravity;
}
