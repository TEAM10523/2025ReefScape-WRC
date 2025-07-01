package frc.robot.subsystems.vision;

import java.util.HashMap;

import frc.robot.util.BlackholeVision.config;
import frc.robot.util.BlackholeVision.poseObservation;

public interface VisionIO {
    class VisionInputs {
        poseObservation[] poseObservations = new poseObservation[0];
    }

    default void setConfig(HashMap<String, config> configs){}

    default void updateInputs(VisionInputs inputs) {}

}
