package frc.robot.subsystems.vision;

import frc.robot.util.BlackholeVision.camera;
import frc.robot.util.BlackholeVision.config;
import frc.robot.util.BlackholeVision.device;
import frc.robot.util.BlackholeVision.poseObservation;
import java.util.HashMap;

public class VisionIOBlackholeVision implements VisionIO {
  camera[] cameras;
  device visionDevice;
  String deviceId;

  public VisionIOBlackholeVision(String deviceId) {
    this.deviceId = deviceId;
  }

  @Override
  public void setConfig(HashMap<String, config> configs) {
    cameras = new camera[configs.size()];
    int i = 0;
    for (String id : configs.keySet()) {
      cameras[i] = new camera(id, configs.get(id));
      i++;
    }
    visionDevice = new device(deviceId, cameras, "2025-reefscape", 0.1651);
  }

  public poseObservation[] getPoseObservations() {
    poseObservation[] poseObservations = new poseObservation[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      poseObservations[i] = cameras[i].getRobotPose();
    }
    return poseObservations;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.poseObservations = getPoseObservations();
  }
}
