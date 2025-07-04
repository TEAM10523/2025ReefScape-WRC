// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.BlackholeVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.NoSuchElementException;

/** Add your docs here. */
public class PoseEstimator {
  public TimeInterpolatableBuffer<Pose2d> odometryBuffer = TimeInterpolatableBuffer.createBuffer(2);
  public TimeInterpolatableBuffer<Pose2d> robotPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(2);
  private Matrix<N3, N1> odometryStateStdDevs;
  private Matrix<N3, N1> processNoisePerUnit;
  private Rotation2d lastGyroYaw = new Rotation2d(0);
  private SwerveModulePosition[] lastModulePositions;
  private Pose2d estimatedPose = new Pose2d();
  private Pose2d odometryPose = new Pose2d();
  private SwerveDriveKinematics kinematics;

  public PoseEstimator(
      SwerveDriveKinematics kinematics,
      Matrix<N3, N1> odometryStateStdDevs,
      Matrix<N3, N1> processNoisePerUnit) {
    this.kinematics = kinematics;
    this.odometryStateStdDevs = odometryStateStdDevs;
    this.processNoisePerUnit = processNoisePerUnit;
  }

  public void updateDrive(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[modulePositions.length];
    if (lastModulePositions == null) {
      lastModulePositions = modulePositions;
    }
    for (int i = 0; i < modulePositions.length; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters),
              modulePositions[i].angle);
    }
    Twist2d twist = kinematics.toTwist2d(wheelDeltas);

    if (gyroYaw != null) {
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());

      lastGyroYaw = gyroYaw;
    }
    SmartDashboard.putNumberArray(
        "wheelDeltas",
        new Double[] {
          wheelDeltas[0].angle.getRadians(), wheelDeltas[0].distanceMeters,
          wheelDeltas[1].angle.getRadians(), wheelDeltas[1].distanceMeters,
          wheelDeltas[2].angle.getRadians(), wheelDeltas[2].distanceMeters,
          wheelDeltas[3].angle.getRadians(), wheelDeltas[3].distanceMeters,
        });

    lastModulePositions = modulePositions;

    estimatedPose = estimatedPose.exp(twist);
    odometryPose = odometryPose.exp(twist);

    odometryBuffer.addSample(Timer.getFPGATimestamp(), odometryPose);

    odometryStateStdDevs =
        odometryStateStdDevs.plus(
            new Matrix<>(
                VecBuilder.fill(
                    Math.max(Math.abs(twist.dx) * (processNoisePerUnit.get(0, 0)), 0.001),
                    Math.max(Math.abs(twist.dy) * (processNoisePerUnit.get(1, 0)), 0.001),
                    Math.max(Math.abs(twist.dtheta) * (processNoisePerUnit.get(2, 0)), 0))));
  }

  public Matrix<N3, N3> getP(Matrix<N3, N1> stdDevs) {
    Matrix<N3, N3> P = new Matrix<>(Nat.N3(), Nat.N3());

    for (int row = 0; row < 3; ++row) {
      double stdDev = stdDevs.get(row, 0);
      P.set(row, row, stdDev * stdDev);
    }
    return P;
  }

  public Matrix<N3, N3> calculateKalmanGain(Matrix<N3, N1> measurementStdDevs) {

    Matrix<N3, N3> P = getP(odometryStateStdDevs);

    Matrix<N3, N3> R = getP(measurementStdDevs);
    SmartDashboard.putNumberArray(
        "odometryStateStdDevs",
        new double[] {
          P.get(0, 0), P.get(1, 1), P.get(2, 2),
        });
    SmartDashboard.putNumberArray(
        "measurementStdDevs",
        new double[] {
          R.get(0, 0), R.get(1, 1), R.get(2, 2),
        });
    // Innovation covariance S = P + R (since we assume an identity observation matrix H)
    Matrix<N3, N3> S = P.plus(R);

    // Compute the inverse of S
    Matrix<N3, N3> SInv = S.plus(Matrix.eye(Nat.N3()).times(1e-10)).inv();

    // Kalman gain: K = P * S^{-1}
    Matrix<N3, N3> K = P.times(SInv);

    return K;
  }

  public boolean poseIsValid(Pose2d visionPose, Pose2d samplePose) {
    return Math.abs(visionPose.getRotation().minus(samplePose.getRotation()).getDegrees())
        < 10; // TODO
    // return true;
  }

  public void addVisionObservation(poseObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (odometryBuffer.getInternalBuffer().lastKey() - 2 > observation.FPGATimestamp) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = odometryBuffer.getSample(observation.FPGATimestamp);
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);

    var visionStdDevs =
        observation.stdDevs.plus(
            new Matrix<>(
                VecBuilder.fill(
                    Math.abs(sampleToOdometryTransform.getX()) * processNoisePerUnit.get(0, 0),
                    Math.abs(sampleToOdometryTransform.getY()) * processNoisePerUnit.get(1, 0),
                    Math.abs(sampleToOdometryTransform.getRotation().getRadians())
                        * processNoisePerUnit.get(2, 0))));

    var VisionPose = observation.pose.toPose2d().plus(sampleToOdometryTransform);
    SmartDashboard.putString("VisionPose1", VisionPose.toString());
    SmartDashboard.putString(" sample", sample.get().toString());

    SmartDashboard.putBoolean("pose is valid", poseIsValid(VisionPose, sample.get()));

    if (!poseIsValid(VisionPose, sample.get())) {
      return;
    }

    Matrix<N3, N3> K = calculateKalmanGain(visionStdDevs);
    SmartDashboard.putNumberArray(
        "K",
        new double[] {
          K.get(0, 0), K.get(1, 1), K.get(2, 2),
        });
    Transform2d innovation = new Transform2d(estimatedPose, VisionPose);

    Transform2d scaledInnovation =
        new Transform2d(
            innovation.getX() * K.get(0, 0),
            innovation.getY() * K.get(1, 1),
            innovation.getRotation().times(K.get(2, 2)));

    estimatedPose = estimatedPose.plus(scaledInnovation);

    // Define the measurement matrix H (Identity for direct measurement)
    Matrix<N3, N3> H = Matrix.eye(Nat.N3());

    // Update the covariance: P' = (I - K H) P
    Matrix<N3, N3> I = Matrix.eye(Nat.N3());
    Matrix<N3, N3> KH = K.times(H);
    Matrix<N3, N3> P_new = (I.minus(KH)).times(getP(odometryStateStdDevs));
    SmartDashboard.putNumberArray(
        "P_new",
        new double[] {
          P_new.get(0, 0), P_new.get(1, 1), P_new.get(2, 2),
        });
    odometryStateStdDevs =
        new Matrix<>(
            VecBuilder.fill(
                (1 - K.get(0, 0)) * odometryStateStdDevs.get(0, 0),
                (1 - K.get(1, 1)) * odometryStateStdDevs.get(1, 0),
                (1 - K.get(2, 2)) * odometryStateStdDevs.get(2, 0)));
  }

  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
    // frame of rotation
    estimatedPose = pose;
    odometryPose = pose;
    odometryBuffer.clear();
  }

  public void setRotation(Rotation2d rotation) {
    estimatedPose = new Pose2d(estimatedPose.getX(), estimatedPose.getY(), rotation);
    odometryPose = new Pose2d(odometryPose.getX(), odometryPose.getY(), rotation);
    odometryBuffer.clear();
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public Pose2d getEstimatedPoseWithTime(double timeStamp) { // FIXME logical problem
    if (timeStamp > Timer.getFPGATimestamp()) {
      return estimatedPose;
    }
    double desireTime = timeStamp;
    try {
      if (odometryBuffer.getInternalBuffer().lastKey() - 2 > desireTime) {
        return estimatedPose;
      }
    } catch (NoSuchElementException ex) {
      return estimatedPose;
    }
    // Get odometry based pose at timestamp
    var sample = odometryBuffer.getSample(desireTime);
    if (sample.isEmpty()) {
      // exit if not there
      return estimatedPose;
    }

    return estimatedPose.plus(new Transform2d(odometryPose, sample.get()));
  }
}
