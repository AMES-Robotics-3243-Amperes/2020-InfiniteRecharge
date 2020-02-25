package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class JoystUtil
{
    /**
   * Given both left and right steering, returns the average of the two if they're
   * really close together.
   * <p>
   * Useful for trying to drive in a straight line.
   */
  public static double matchZone(double steer1, double steer2) {
    double matchZoneRadius = 0.09;

    double avgSteer = (steer1 + steer2) / 2.0;
    if (Constants.TEST_VERSION)
      SmartDashboard.putNumber("avgSteer", avgSteer);
    double r = matchZoneRadius / 2.0;

    // The steer1 values past which steer1's unadjusted value should be returned.
    // If avgSteer is >0, steer1 will be returned at 1 and below 0. If <0, at -1 and
    // above 0.
    double lowerBound = (avgSteer - r <= 0) ? -1 : 0;
    double upperBound = (avgSteer + r >= 0) ? 0 : 1;
    // Very slight bias functions to make sure lowerBound and upperBound are
    // respected.
    double upperBoundCorrector = clamp((steer1 - (avgSteer + r)) * ((r) / (upperBound - (avgSteer + r))), 0, r);
    double lowerBoundCorrector = clamp((steer1 - (avgSteer - r)) * ((r) / ((avgSteer - r) - lowerBound)), -r, 0);

    if (Constants.TEST_VERSION) {
      SmartDashboard.putNumber("upperBoundConnector", upperBoundCorrector);
      SmartDashboard.putNumber("lowerBoundConnector", lowerBoundCorrector);
    }

    // A y=x function with a plateau in the middle. The plateau is at avgSteer and
    // causes values to 'snap' to it.
    // Either lowerBoundCorrector or upperBoundCorrector is added, depending on
    // whether steer1 is between avgSteer
    // and lowerBound, or between avgSteer and upperBound.
    double result = Math.signum(steer1 - avgSteer) * Math.max(0, Math.abs(steer1 - avgSteer) - r) + avgSteer
        + ((steer1 > avgSteer + r) ? upperBoundCorrector : lowerBoundCorrector);

    return result;
    // return (Math.abs(avgSteer-steer1) <= r) ?avgSteer :steer1; Without smoothing;
    // don't use.
  }

  private static double clamp(double a, double min, double max) {
    double realMin = Math.min(min, max);
    double realMax = Math.max(min, max);
    return Math.min(realMax, Math.max(realMin, a));
  }

  private static double lerp(double a, double b, double f) {
    return a + f * (b - a);
  }

  public static double deadZone(double dead) {
    double deadZoneRadius = 0.09;
    // This makes input increase smoothly, instead of jumping up slightly as soon as
    // 0.09 is passed.
    return Math.signum(dead) * Math.max(0, Math.abs(dead * (1 + deadZoneRadius)) - deadZoneRadius);
  }

  public static double scaleZone(double scale) {
    return Math.pow(scale, 3);
  }
}