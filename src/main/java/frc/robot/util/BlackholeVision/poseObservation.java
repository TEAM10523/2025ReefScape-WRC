package frc.robot.util.BlackholeVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class poseObservation {
    public Pose3d pose;
    public Matrix<N3, N1> stdDevs;
    public double FPGATimestamp;
    public poseObservation(Pose3d pose, Matrix<N3, N1> stdDevs, double FPGATimestamp){
        this.pose = pose;
        this.stdDevs = stdDevs;
        this.FPGATimestamp = FPGATimestamp;
    }
}
