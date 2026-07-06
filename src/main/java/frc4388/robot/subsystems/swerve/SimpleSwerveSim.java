package frc4388.robot.subsystems.swerve;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc4388.robot.subsystems.vision.VisionIO.PoseObservation;

public class SimpleSwerveSim implements SwerveIO {
    private Pose2d pose = new Pose2d();
    private Pose2d lastPose = pose;
    private double vx = 0.0;
    private double vy = 0.0;
    private double omega = 0.0;

    private long lastTimeNs = System.nanoTime();

    public SimpleSwerveSim() {
    }

    public synchronized void setControl(SwerveRequest ctrl) {
        if (ctrl == null) return;

        if (ctrl instanceof SwerveRequest.FieldCentricFacingAngle facingAngle) {
        vx = facingAngle.VelocityX;
        vy = facingAngle.VelocityY;

        double currentAngle = pose.getRotation().getRadians();
        double targetAngle  = facingAngle.TargetDirection.getRadians();
        double error = targetAngle - currentAngle;

            error = Math.atan2(Math.sin(error), Math.cos(error));

            double kP = 5.0;
            omega = error * kP;
            return;
        }

        if (ctrl instanceof SwerveRequest.FieldCentric fc) {
            vx = fc.VelocityX;
            vy = fc.VelocityY;
            omega = fc.RotationalRate;
            return;
        }

        if (ctrl instanceof SwerveRequest.RobotCentric rc) {
            double cos = pose.getRotation().getCos();
            double sin = pose.getRotation().getSin();
            double vxRobot = rc.VelocityX;
            double vyRobot = rc.VelocityY;
            vx = vxRobot * cos - vyRobot * sin;
            vy = vxRobot * sin + vyRobot * cos;
            omega = rc.RotationalRate;
            return;
        }

        if (ctrl instanceof SwerveRequest.SwerveDriveBrake) {
            vx = 0; vy = 0; omega = 0;
            return;
        }

        ChassisSpeeds cs = tryGetSpeedsField(ctrl);
        if (cs != null) {
            vx = cs.vxMetersPerSecond;
            vy = cs.vyMetersPerSecond;
            omega = cs.omegaRadiansPerSecond;
        }
    }

    private ChassisSpeeds tryGetSpeedsField(SwerveRequest ctrl) {
        try {
            java.lang.reflect.Field f = ctrl.getClass().getDeclaredField("Speeds");
            f.setAccessible(true);
            Object o = f.get(ctrl);
            if (o instanceof ChassisSpeeds) {
                return (ChassisSpeeds) o;
            }
        } catch (NoSuchFieldException nsf) {
        } catch (IllegalAccessException iae) {
        } catch (SecurityException se) {
        }
        return null;
    }

    private double tryGetDoubleField(Object obj, Class<?> cls, String... names) {
        for (String n : names) {
            try {
                java.lang.reflect.Field f = cls.getDeclaredField(n);
                f.setAccessible(true);
                Object val = f.get(obj);
                if (val instanceof Number) {
                    return ((Number) val).doubleValue();
                }
            } catch (NoSuchFieldException nsf) {
            } catch (IllegalAccessException iae) {
            } catch (Throwable t) {
            }
        }
        return 0.0;
    }

    
    public synchronized void pointAtXY(double x, double y) {
        Translation2d target = new Translation2d(x, y);
        Translation2d toTarget = target.minus(pose.getTranslation());
        if (toTarget.getNorm() < 1e-9) return;
        Rotation2d desired = toTarget.getAngle();
        pose = new Pose2d(pose.getTranslation(), desired);
        lastPose = pose;
        vx = 0.0;
        vy = 0.0;
        omega = 0.0;
    }

    @Override
    public synchronized void updateInputs(SwerveState state) {
        if (state == null) return;

        long now = System.nanoTime();
        double dt = Math.max(1e-6, (now - lastTimeNs) / 1.0e9);
        lastTimeNs = now;

        lastPose = pose;
        double dxField;
        double dyField;
        
        if (DriverStation.isAutonomous()) {
            double dxRobot = vx * dt;
            double dyRobot = vy * dt;

            Rotation2d r = pose.getRotation();
            double cos = r.getCos();
            double sin = r.getSin();

            dxField = dxRobot * cos - dyRobot * sin;
            dyField = dxRobot * sin + dyRobot * cos;

        } else {
            dxField = vx * dt;
            dyField = vy * dt;

        }


        double dTheta = omega * dt;

        Translation2d newTrans = pose.getTranslation().plus(new Translation2d(dxField, dyField));
        Rotation2d newRot = pose.getRotation().plus(Rotation2d.fromRadians(dTheta));
        pose = new Pose2d(newTrans, newRot);

        state.lastPose = lastPose;
        state.currentPose = pose;
        state.speeds = new ChassisSpeeds(vx, vy, omega);
        state.odometryRate = dt;
    }

    @Override
    public synchronized void resetPose(Pose2d p) {
        if (p == null) return;
        pose = p;
        lastPose = p;
        lastTimeNs = System.nanoTime();
    }

    @Override
    public synchronized void tareEverything() {
        pose = new Pose2d();
        lastPose = pose;
        vx = 0.0;
        vy = 0.0;
        omega = 0.0;
        lastTimeNs = System.nanoTime();
    }

    @Override
    public synchronized void addVisionMeasurement(List<PoseObservation> poses) {
        if (poses == null || poses.isEmpty()) return;
        Pose2d visPose = poses.get(poses.size() - 1).pose();
        if (visPose != null) {
            pose = visPose;
            lastPose = visPose;
        }
    }

    public synchronized void pointAt(Translation2d target) {
        if (target == null) return;
        Translation2d toTarget = target.minus(pose.getTranslation());
        if (toTarget.getNorm() < 1e-9) return;
        Rotation2d desired = toTarget.getAngle();
        pose = new Pose2d(pose.getTranslation(), desired);
        lastPose = pose;
        omega = 0.0;
    }

    @Override
    public synchronized void setLimits(double limitInAmps) {
    }
}
