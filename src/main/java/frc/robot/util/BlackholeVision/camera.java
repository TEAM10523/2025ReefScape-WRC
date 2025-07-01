package frc.robot.util.BlackholeVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

public class camera {
  DoubleArrayPublisher cameraMatrix_publisher;
  DoubleArrayPublisher distortionCoeffs_publisher;
  DoubleArrayPublisher cameraPose_publisher;
  IntegerArrayPublisher resolution_publisher;
  DoublePublisher exposure_publisher;
  DoublePublisher gain_publisher;
  DoublePublisher maxFPS_publisher;

  DoubleArraySubscriber multiTagPose_subscriber;
  DoubleSubscriber multiTagError_subscriber;
  DoubleSubscriber latency_subscriber;
  config cameraConfig;

  String cameraPort;

  public camera(String cameraPort, config cameraConfig) {
    this.cameraPort = cameraPort;
    this.cameraConfig = cameraConfig;
  }

  public String getPort() {
    return cameraPort;
  }

  public static <T> Stream<T> flattenStream(T[][] arrays) {

    // Create an empty list to collect the stream
    List<T> list = new ArrayList<>();

    // Using forEach loop
    // convert the array into stream
    // and add the stream into list
    for (T[] array : arrays) {
      Arrays.stream(array).forEach(list::add);
    }

    // Convert the list into Stream and return it
    return list.stream();
  }

  public void setup(NetworkTable deviceTable) {
    cameraMatrix_publisher =
        deviceTable.getDoubleArrayTopic(cameraPort + "/cameraMatrix").publish();
    distortionCoeffs_publisher =
        deviceTable.getDoubleArrayTopic(cameraPort + "/distortionCoeffs").publish();
    cameraPose_publisher = deviceTable.getDoubleArrayTopic(cameraPort + "/cameraPose").publish();
    resolution_publisher = deviceTable.getIntegerArrayTopic(cameraPort + "/resolution").publish();
    exposure_publisher = deviceTable.getDoubleTopic(cameraPort + "/exposure").publish();
    gain_publisher = deviceTable.getDoubleTopic(cameraPort + "/gain").publish();
    maxFPS_publisher = deviceTable.getDoubleTopic(cameraPort + "/maxFPS").publish();

    multiTagPose_subscriber =
        deviceTable
            .getDoubleArrayTopic(cameraPort + "/multiTagPose")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    multiTagError_subscriber =
        deviceTable
            .getDoubleTopic(cameraPort + "/multiTagError")
            .subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    latency_subscriber =
        deviceTable
            .getDoubleTopic(cameraPort + "/latency")
            .subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

    cameraMatrix_publisher.set(cameraConfig.cameraMatrix);
    distortionCoeffs_publisher.set(cameraConfig.distortionCoeffs);

    var cameraPose = cameraConfig.cameraPose;
    var cameraRotation = cameraPose.getRotation();
    cameraPose_publisher.set(
        new double[] {
          cameraPose.getX(),
          cameraPose.getY(),
          cameraPose.getZ(),
          cameraRotation.getX(),
          cameraRotation.getY(),
          cameraRotation.getZ()
        });

    resolution_publisher.set(cameraConfig.resolution);
    exposure_publisher.set(cameraConfig.exposure);
    gain_publisher.set(cameraConfig.gain);
    maxFPS_publisher.set(cameraConfig.maxFPS);
  }

  public poseObservation getRobotPose() {
    double[] poseList = multiTagPose_subscriber.get(new double[] {0, 0, 0, 0, 0, 0});
    double error = multiTagError_subscriber.get(-1);

    Matrix<N3, N1> stdDevs = new Matrix<>(VecBuilder.fill(
      Math.pow(error * 100, 2), 
      Math.pow(error * 100, 2), 
      Math.pow(error * 1000, 2)));
    if (error < 0) {
      stdDevs = null;
    }

    double latency = latency_subscriber.get(0); // TODO
    double FPGATimestamp = Timer.getFPGATimestamp() - 0.25;
    // double t = multiTagPose_subscriber.getLastChange(); #TODO
    return new poseObservation(
        new Pose3d(
            poseList[0],
            poseList[1],
            poseList[2],
            new Rotation3d(poseList[3], poseList[4], poseList[5])),
        stdDevs,
        FPGATimestamp);
  }
}
