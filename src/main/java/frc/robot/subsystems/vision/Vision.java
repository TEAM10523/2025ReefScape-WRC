// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.VisionInputs;
import frc.robot.util.BlackholeVision.config;
import frc.robot.util.BlackholeVision.poseObservation;
import java.util.HashMap;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  VisionIO[] visionIOs;

  int num_cameras = 0;
  VisionInputs[] visionInputs;

  private Drive drive;

  public Vision(VisionIO[] ios, HashMap<String, config>[] configs, Drive drive) {
    this.visionIOs = ios;
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].setConfig(configs[i]);
      num_cameras += configs[i].size();
    }
    visionInputs = new VisionInputs[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionInputs[i] = new VisionInputs();
    }
    this.drive = drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(visionInputs[i]);
    }

    poseObservation[] poseObservations = getPoseObservations();
    for(poseObservation poseObservation : poseObservations){
      if(poseObservation == null){
        continue;
      }

      if (Math.abs(poseObservation.pose.getZ()) > VisionConstants.heightTolerance
          || Math.abs(poseObservation.pose.getRotation().getY()) > VisionConstants.pitchTolerance) {
        continue;
      }

      var poseSample = drive.getPoseSample(poseObservation.FPGATimestamp);

      if (!poseSample.isPresent()) {
        continue;
      }

      var poseAtTime = poseSample.get();

      if (Math.abs(
              poseAtTime
                  .getRotation()
                  .minus(poseObservation.pose.toPose2d().getRotation())
                  .getRadians())
          > VisionConstants.yawTolerance) {
        continue;
      }

      drive.addVisionMeasurement(
          poseObservation.pose.toPose2d(), num_cameras, poseObservation.stdDevs);
    }
  }

  public poseObservation[] getPoseObservations() {
    poseObservation[] observations = new poseObservation[num_cameras];
    int n = 0;
    for (int i = 0; i < visionIOs.length; i++) {
      poseObservation[] singleObservations = visionInputs[i].poseObservations;
      for (int j = 0; j < singleObservations.length; j++) {
        observations[n] = singleObservations[j];
        n++;
      }
    }
    return observations;
  }
}
