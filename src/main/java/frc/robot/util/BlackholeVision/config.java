// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.BlackholeVision;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class config {
  public double[] cameraMatrix;
  public double[] distortionCoeffs;
  public Pose3d cameraPose;
  public long[] resolution;
  public double exposure;
  public double gain;
  public double maxFPS;
  public boolean enableTag;
  public boolean enableObj;

  public config(
      double[] cameraMatrix,
      double[] distortionCoeffs,
      Pose3d cameraPose,
      long[] resolution,
      double exposure,
      double gain,
      double maxFPS,
      boolean enableTag,
      boolean enableObj) {
    this.cameraMatrix = cameraMatrix;
    this.distortionCoeffs = distortionCoeffs;
    this.cameraPose = cameraPose;
    this.resolution = resolution;
    this.exposure = exposure;
    this.gain = gain;
    this.maxFPS = maxFPS;
    this.enableTag = enableTag;
    this.enableObj = enableObj;
  }
}
