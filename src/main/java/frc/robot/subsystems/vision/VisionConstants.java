package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.BlackholeVision.config;
import java.util.HashMap;

public class VisionConstants {

  public static final VisionIO[] ios =
      new VisionIOBlackholeVision[] {new VisionIOBlackholeVision("BlackholeVision")};
  public static HashMap<String, config> backConfig =
      new HashMap<>() {
        {
          put(
              "camera_top_right",
              new config(
                  new double[] {
                    915.64624796, 0., 711.42068653, 0., 914.59523767, 318.54322594, 0., 0., 1.
                  },
                  new double[] {
                    3.87500264e-02, -5.57462450e-02, 8.69038755e-05, 1.82044089e-03, -1.19112189e-03
                  },
                  new Pose3d(
                      -0.188,
                      -0.222,
                      0.296,
                      new Rotation3d(Math.toRadians(-25), 0, Math.toRadians(180 - 32.5))),
                  // new Pose3d(-0.328, 0.16126, 0.215, new Rotation3d(0,0, 2.617998)),

                  new long[] {720, 1280},
                  20,
                  1,
                  60));
          put(
              "camera_bottom_right",
              new config(
                  // new double[]{731.12673711,0,689.82988468,0,731.32038991,294.39403205,0,0,1} ,
                  // new double[]{0.13468574,-0.01416523,0.00054,-0.00029907,0.10044209
                  // },
                  new double[] {
                    909.2490064, 0., 689.83200643, 0., 910.52636511, 338.18824425, 0., 0., 1.
                  },
                  new double[] {
                    2.48776830e-02, -1.65343141e-02, 3.79979921e-05, 5.64711091e-04, -3.30895103e-02
                  },
                  new Pose3d(
                      -0.186,
                      0.223,
                      0.315,
                      new Rotation3d(Math.toRadians(25), 0, Math.toRadians(180 + 32.5))),
                  // new Pose3d(-0.328, 0.16126, 0.215, new Rotation3d(0,0, 2.617998)),

                  new long[] {720, 1280},
                  20,
                  1,
                  60));
        }
      };
  public static final HashMap<String, config>[] configs = new HashMap[] {backConfig};

  public static final double heightTolerance = 0.4;
  public static final double yawTolerance = 0.1;
  public static final double pitchTolerance = 0.1;
}
