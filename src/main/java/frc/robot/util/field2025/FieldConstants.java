package frc.robot.util.field2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
  public static final double fieldLength = 17.548;
  public static final double filedWidth = 8.052;

  public static final Pose2d reefBlueA = new Pose2d(3.256, 4.192, new Rotation2d(0));
  public static final Pose2d reefBlueB = new Pose2d(3.256, 3.844, new Rotation2d(0));
  public static final Pose2d reefBlueC = new Pose2d(3.743, 3.0368, new Rotation2d(1.0471975));
  public static final Pose2d reefBlueD = new Pose2d(4.026, 2.875, new Rotation2d(1.0471975));
  public static final Pose2d reefBlueE = new Pose2d(4.963, 2.873, new Rotation2d(2.09439));
  public static final Pose2d reefBlueF = new Pose2d(5.251, 3.039, new Rotation2d(2.09439));
  public static final Pose2d reefBlueG = new Pose2d(5.718, 3.844, new Rotation2d(3.1415926));
  public static final Pose2d reefBlueH = new Pose2d(5.718, 4.192, new Rotation2d(3.1415926));
  public static final Pose2d reefBlueI = new Pose2d(5.2472, 5.0151, new Rotation2d(-2.09439));
  public static final Pose2d reefBlueJ = new Pose2d(4.9626, 5.1740, new Rotation2d(-2.09439));
  public static final Pose2d reefBlueK = new Pose2d(4.0160, 5.1768, new Rotation2d(-1.0471975));
  public static final Pose2d reefBlueL = new Pose2d(3.7291, 5.0085, new Rotation2d(-1.0471975));

  public static final pairReef reefBack =
      new pairReef(
          new Pose2d(3.180, 4.0235, new Rotation2d(0)), new Pose2d[] {reefBlueB, reefBlueA}, 2);
  public static final pairReef reefBackRight =
      new pairReef(
          new Pose2d(3.823, 2.881, new Rotation2d(1.0471975)),
          new Pose2d[] {reefBlueD, reefBlueC},
          3);
  public static final pairReef reefFrontRight =
      new pairReef(
          new Pose2d(5.154, 2.8735, new Rotation2d(2.09439)),
          new Pose2d[] {reefBlueE, reefBlueF},
          2);
  public static final pairReef reefFront =
      new pairReef(
          new Pose2d(5.815, 4.0265, new Rotation2d(3.1415926)),
          new Pose2d[] {reefBlueG, reefBlueH},
          3);
  public static final pairReef reefFrontLeft =
      new pairReef(
          new Pose2d(5.158, 5.1825, new Rotation2d(-2.09439)),
          new Pose2d[] {reefBlueI, reefBlueJ},
          2);
  public static final pairReef reefBackLeft =
      new pairReef(
          new Pose2d(3.824, 5.171, new Rotation2d(-1.0471975)),
          new Pose2d[] {reefBlueL, reefBlueK},
          3);

  public static final pairReef[] pairReefs =
      new pairReef[] {
        reefBack, reefBackRight, reefFrontRight, reefFront, reefFrontLeft, reefBackLeft
      };
}
