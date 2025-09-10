
package frc4388.utility.controller;

import static frc4388.robot.constants.Constants.OIConstants.LEFT_AXIS_DEADBAND;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class DeadbandedXboxController extends XboxController {
  public DeadbandedXboxController(int port) { super(port); }
  
  @Override public double getLeftX() { return getLeft().getX(); }
  @Override public double getLeftY() { return getLeft().getY(); }
  @Override public double getRightX() { return getRight().getX(); }
  @Override public double getRightY() { return getRight().getY(); }

  public Translation2d getLeft() { return skewToDeadzonedCircle(super.getLeftX(), super.getLeftY()); }
  public Translation2d getRight() { return skewToDeadzonedCircle(-super.getRightX(), super.getRightY()); }

  public static Translation2d skewToDeadzonedCircle(double x, double y) {
    Translation2d translation2d = new Translation2d(x, y);
    double magnitude = translation2d.getNorm();

    if (magnitude < LEFT_AXIS_DEADBAND) return new Translation2d(0,0);

    return translation2d;
  }
}