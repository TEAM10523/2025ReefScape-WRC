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
                    731.12673711, 0, 689.82988468, 0, 731.32038991, 294.39403205, 0, 0, 1
                  },
                  new double[] {0.13468574, -0.01416523, 0.00054, -0.00029907, 0.10044209},
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
                    734.6914699, 0., 697.69124948, 0., 734.76207668, 317.22929688, 0, 0, 1
                  },
                  new double[] {
                    1.28929702e-01, -2.33682572e-01, 7.34973535e-05, 2.48566170e-04, 8.76871677e-02
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

  public static final double heightTolerance = 0.3;
  public static final double yawTolerance = 0.1;
  public static final double pitchTolerance = 0.1;
}
