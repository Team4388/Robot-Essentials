// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {
  
  private PhotonCamera cam;
  private PhotonPoseEstimator photonPoseEstimator;

  private boolean lightOn;

  /** Creates a new Limelight. */
  public Limelight() {
    cam = new PhotonCamera(VisionConstants.NAME);
    cam.setDriverMode(false);
  }

  public void setLEDs(boolean on) {
    lightOn = on;
    cam.setLED(lightOn ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void toggleLEDs() {
    lightOn = !lightOn;
    cam.setLED(lightOn ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void setDriverMode(boolean driverMode) {
    cam.setDriverMode(driverMode);
  }

  public void setToLimePipeline() {
    cam.setPipelineIndex(1);
    setLEDs(true);
  }

  public void setToAprilPipeline() {
    cam.setPipelineIndex(0);
    setLEDs(false);
  }

  public PhotonTrackedTarget getAprilPoint() {
    if (!cam.isConnected()) return null;

    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) return null;

    return result.getBestTarget();
  }

  private List<TargetCorner> getAprilCorners() {
    if (!cam.isConnected()) return null;

    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) return null;

    return result.getBestTarget().getDetectedCorners();
  }

  public double getAprilSkew() {
    List<TargetCorner> corners = getAprilCorners();
    ArrayList<TargetCorner> bottomSide = getAprilBottomSide(corners);

    if (bottomSide == null) return 0;

    TargetCorner bottomRight = bottomSide.get(0).x > bottomSide.get(1).x ? bottomSide.get(0) : bottomSide.get(1);
    TargetCorner bottomLeft = bottomRight.x == bottomSide.get(0).x ? bottomSide.get(1) : bottomSide.get(0);

    return bottomLeft.y - bottomRight.y;
  }

  private ArrayList<TargetCorner> getAprilBottomSide(List<TargetCorner> box) {
    if (box == null) return null;

    ArrayList<TargetCorner> bottomSide = new ArrayList<>();

    TargetCorner l1 = new TargetCorner(-1, -1);
    TargetCorner l2 = new TargetCorner(-1, -1);
    
    for (TargetCorner c : box) {
      if (c.y > l1.y) l1 = c;
    }

    for (TargetCorner c : box) {
      if (c.y == l1.y) continue;
      if (c.y > l2.y) l2 = c;
    }

    bottomSide.add(l1);
    bottomSide.add(l2);

    return bottomSide;
  }

  public double getDistanceToApril() {    
    PhotonTrackedTarget aprilPoint = getAprilPoint();
    if (aprilPoint == null) return -1;

    double aprilHeight = VisionConstants.APRIL_HEIGHT - VisionConstants.LIME_HEIGHT;
    double theta = 35.0 + aprilPoint.getPitch();

    double distanceToApril = aprilHeight / Math.tan(Math.toRadians(theta));
    return distanceToApril;
  }

  public PhotonTrackedTarget getLowestTape() {
    if (!cam.isConnected()) return null;

    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) return null;

    ArrayList<PhotonTrackedTarget> points = (ArrayList<PhotonTrackedTarget>) result.getTargets();

    PhotonTrackedTarget lowest = points.get(0);
    for (PhotonTrackedTarget point : points) {
      if (point.getPitch() < lowest.getPitch()) {
        lowest = point;
      }
    }

    return lowest;
  }

  public double getDistanceToTape() {    
    PhotonTrackedTarget tapePoint = getLowestTape();
    if (tapePoint == null) return -1;

    double tapeHeight = VisionConstants.MID_TAPE_HEIGHT - VisionConstants.LIME_HEIGHT;
    double theta = 35.0 + tapePoint.getPitch();

    double distanceToTape = tapeHeight / Math.tan(Math.toRadians(theta));
    return distanceToTape;
  }

  @Override
  public void periodic() {}
}
