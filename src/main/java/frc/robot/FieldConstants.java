package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
  // inspired by 6940
  static Translation2d BlueReefCenterPos = new Translation2d(4.489323, 8.0518 / 2.);
  static Translation2d DReefTranslation12 = new Translation2d(1.2333, 0.1661);
  static Translation2d DAlgaeTranslation12 = new Translation2d(1.309323, 0);
  public static final Translation2d FieldCenter = new Translation2d(17.548225 / 2, 8.0518 / 2.);

  public static Pose2d rotateAroundCenter(Pose2d pose, Translation2d centre, Rotation2d rotation) {
    return new Pose2d(
        pose.getTranslation().rotateAround(centre, rotation), pose.getRotation().plus(rotation));
  }

  public static Pose2d generateReefPose(int index) {
    Translation2d t = FieldConstants.BlueReefCenterPos;
    Translation2d dt = FieldConstants.DReefTranslation12;
    if (index % 2 == 1) {
      dt = new Translation2d(dt.getX(), -dt.getY());
    }
    t = t.plus(dt);

    Rotation2d r = new Rotation2d(Math.PI);
    Rotation2d dr = Rotation2d.fromDegrees((double) ((index + 1) / 2) * 60.);

    t = t.rotateAround(FieldConstants.BlueReefCenterPos, dr);
    r = r.plus(dr);

    Pose2d res = new Pose2d(t, r);

    // if (DriverStation.getAlliance().get() == Alliance.Red) {
    //   // t = t.rotateAround(FieldConstants.FieldCenter, Rotation2d.fromDegrees(180));
    //   // r = r.plus(Rotation2d.fromDegrees(180));
    //   res = FieldConstants.rotateAroundCenter(res, FieldConstants.FieldCenter,
    // Rotation2d.k180deg);
    // }

    if (res == null) {
      DriverStation.reportWarning("1111111111111111111", false);
    }

    return res;
  }

  public static Pose2d generateAlgaePose(int index) {
    Translation2d t = FieldConstants.BlueReefCenterPos;
    Translation2d dt = FieldConstants.DAlgaeTranslation12;
    t = t.plus(dt);

    Rotation2d r = new Rotation2d(Math.PI);
    Rotation2d dr = Rotation2d.fromDegrees((double) (index) * 60.);

    t = t.rotateAround(FieldConstants.BlueReefCenterPos, dr);
    r = r.plus(dr);

    Pose2d res = new Pose2d(t, r);

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      // t = t.rotateAround(FieldConstants.FieldCenter, Rotation2d.fromDegrees(180));
      // r = r.plus(Rotation2d.fromDegrees(180));
      res = FieldConstants.rotateAroundCenter(res, FieldConstants.FieldCenter, Rotation2d.k180deg);
    }

    if (res == null) {
      DriverStation.reportWarning("1111111111111111111", false);
    }

    return res;
  }

  public static Translation2d getFromReefCentreTranslation(Pose2d _Pose2d) {
    // Translation2d t = FieldConstants.BlueReefCenterPos.minus(getPose().getTranslation());
    // if(DriverStation.getAlliance().get() == Alliance.Red){
    //     t.rotateBy(new Rotation2d(Math.PI));
    // }
    // return t;

    return DriverStation.getAlliance().get() == Alliance.Blue
        ?
        // FieldConstants.BlueReefCenterPos.minus(getPose().getTranslation()) :
        _Pose2d.getTranslation().minus(FieldConstants.BlueReefCenterPos)
        : FieldConstants.BlueReefCenterPos.rotateAround(
                FieldConstants.FieldCenter, Rotation2d.k180deg)
            .minus(_Pose2d.getTranslation());
  }
  /**
   * @param _Pose2d
   * @param _LRindex 1 is left 2 is right
   * @param _Alliance
   * @return
   */
  public static Pose2d getClosestReefPose(
      Translation2d _Pose2d, int _LRindex, DriverStation.Alliance _Alliance) {
    Pose2d closest = null;
    double minDistance = Double.MAX_VALUE;
    for (int i = _LRindex; i <= 12; i += 2) {
      Pose2d reefPose = generateReefPose(i);
      if (_Alliance == DriverStation.Alliance.Red) {
        reefPose =
            FieldConstants.rotateAroundCenter(
                reefPose, FieldConstants.FieldCenter, Rotation2d.k180deg);
      }
      double distance = _Pose2d.getDistance(reefPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closest = reefPose;
      }
    }
    return closest;
  }
  /**
   * @param _Pose2d
   * @param _LRindex 1 is left 2 is right
   * @param _Alliance
   * @return
   */
  public static Pose2d getClosestAlgaePose(
      Translation2d _Pose2d, DriverStation.Alliance _Alliance) {
    int closest = 1;
    double minDistance = Double.MAX_VALUE;
    for (int i = 1; i <= 12; ++i) {
      Pose2d reefPose = generateReefPose(i);
      if (_Alliance == DriverStation.Alliance.Red) {
        reefPose =
            FieldConstants.rotateAroundCenter(
                reefPose, FieldConstants.FieldCenter, Rotation2d.k180deg);
      }
      double distance = _Pose2d.getDistance(reefPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closest = i;
      }
    }
    return generateAlgaePose(closest);
  }
}
