package frc4388.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import org.littletonrobotics.junction.AutoLogOutput;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.subsystems.vision.RPLidarA1.PolarPoint;
import frc4388.robot.subsystems.vision.RPLidarA1.ScanListener;
import frc4388.utility.configurable.ConfigurableDouble;
import frc4388.utility.status.FaultA1M8;



public class Lidar extends SubsystemBase implements ScanListener {
    // private final Spark m_motor;
    private final RPLidarA1 lidar;

    private ConfigurableDouble speed = new ConfigurableDouble("LiDAR speed", 0.2);

    static
    {
        // This is so libopencv_javaVERSION.so (where version is the 3-digit opencv
        // version) gets loaded.
        try {
            OpenCvLoader.forceLoad();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    // private static final double m_Scan = 0.1;
    public Lidar() {
        // Spark motor = new Spark(0);
        // this.m_motor = motor;
        this.lidar = new RPLidarA1();
        this.lidar.setListener(this);


        // Thread processThread = new Thread(this::pointLoop);
        // processThread.setDaemon(true);
        // processThread.setName("RPLidar-Calc");
        // processThread.start();

        FaultA1M8.addDevice(lidar, "A1M8");
    }

    public Rotation2d getLatestBallAngle() {
        return latestBallAngleDeg;
    }

    public boolean outOfBounds(Translation2d closestBall){
        // Make sure robot doesn't go off the earth
        return true;
    }
    

    @Override
    public void periodic() {
        this.lidar.setSpeed(speed.get());
        SmartDashboard.putString("lidar state", this.lidar.getStatus().toString());
    }

    // Detection constriants: cluster detection
    private static final double ANG_MAX_GAP = 3.; // Degrees
    private static final double DIST_MAX_GAP = 0.04; // Meters

    // Detection constraints: Circle detection 
    private static final double RADIUS_X_COEFF = Units.inchesToMeters(0);
    private static final double RADIUS_Y_COEFF = Units.inchesToMeters(0);
    private static final double RADIUS_OFFSET = Units.inchesToMeters(3);
    private static final double RADIUS_TOLERANCE = Units.inchesToMeters(3);

    private static boolean radiusInTolerance(double x, double y, double radius) {
        double rad_at_position = RADIUS_X_COEFF*x + RADIUS_Y_COEFF*y + RADIUS_OFFSET;

        return Math.abs(rad_at_position - radius) <= RADIUS_TOLERANCE;
    }

    // Window constants
    private static final int WIDTH = 512;
    private static final int HEIGHT = 512;
    private static final int POINT_RAD = 2;


    Translation2d closestBall;
    Translation2d closestBallPrior = null;    

    @AutoLogOutput
    public Translation2d getClosestBall() {
        return closestBall;
    }

   

    private List<Point> point_group = new ArrayList<>();
    private double last_ang = 0;
    private double last_dist = 0;
    private Rotation2d latestBallAngleDeg= new Rotation2d();
    private boolean last_color = false;
    
    Point LIDAR = new Point(WIDTH/2,WIDTH/2);

    @Override
    public void onScanComplete(List<PolarPoint> scan) {

        System.out.println("SCAN: " + scan.size());

        double scale = 0.009;

        List<Translation2d> circlePoints = new ArrayList<>();

        // Mat mat = Mat.zeros(WIDTH, HEIGHT, CvType.CV_8UC3);

        for(PolarPoint point_polar : scan) {
            if(!(point_polar.angle < 30 || point_polar.angle > 330)) {
                continue;
            }

            double ang_rad = Math.toRadians(point_polar.angle); 
            double x = point_polar.distance * Math.cos(ang_rad);
            double y = point_polar.distance * Math.sin(ang_rad);

            // Point point_xy = new Point((WIDTH/2) + (x/scale), (HEIGHT/2) + (y/scale));
            Point point_xy = new Point(x, y);
            
            if(
                Math.abs(last_ang - point_polar.angle) > ANG_MAX_GAP ||
                Math.abs(last_dist - point_polar.distance) > DIST_MAX_GAP
            ) {
                last_color = !last_color;

                if (
                    point_group.size() >= 3
                    // point_group.size() <= POINT_MAX.get()
                ) {
                    // Get points
                    Point p1 = point_group.get(0);
                    Point p2 = point_group.get(point_group.size()/2);
                    Point p3 = point_group.get(point_group.size()-1);

                    // Simplify to var
                    double dx23 = p2.x - p3.x;
                    double dy23 = p2.y - p3.y;

                    double dx13 = p1.x - p3.x;
                    double dy13 = p1.y - p3.y;

                    double dx12 = p1.x - p2.x;
                    double dy12 = p1.y - p2.y;

                    // Calc Determinite
                    double D = p1.x*dy23 - p1.y*dx23 + (p2.x*p3.y - p3.x*p2.y);

                    // The points are in a straight line.
                    if(D == 0) {
                        continue;
                    }

                    // Square distances between each set of 2 points
                    double a_sq = dx23*dx23 + dy23*dy23;
                    double b_sq = dx13*dx13 + dy13*dy13;
                    double c_sq = dx12*dx12 + dy12*dy12;

                    // Calculate the radius
                    double radius = Math.sqrt(a_sq*b_sq*c_sq) / (2 * Math.abs(D));

                    // Square distances between each point and origin
                    double d1 = p1.x*p1.x + p1.y*p1.y;
                    double d2 = p2.x*p2.x + p2.y*p2.y;
                    double d3 = p3.x*p3.x + p3.y*p3.y;

                    // Calculate X and Y
                    double cx = (d1*dy23 - d2*dy13 + d3*dy12)/(2*D);
                    double cy = -(d1*dx23 - d2*dx13 + d3*dx12)/(2*D);



                    if(radiusInTolerance(cx, cy, radius)) {
                        circlePoints.add(new Translation2d(cx, cy));
                    }


                    

                    // FitResult result = TaubinCircleFitter.fit(point_group, Lidar::getRadius);

                    // if(result.rmsError < ERROR_BOUND.get()) {
                    //     circlePoints.add(result.center);
                    //     Imgproc.circle(mat, result.center, (int) (result.radius/scale), new Scalar(255,255,255));
                    // }
                }
                
                point_group.clear();
            }

            point_group.add(point_xy);

            last_ang = point_polar.angle;
            last_dist = point_polar.distance;

            // Point scaledPoint = new Point((WIDTH/2) + (point_xy.x / scale), (WIDTH/2) + (point_xy.y / scale));
            // Imgproc.circle(mat, scaledPoint, POINT_RAD, new Scalar(127,127,127));
        }

        // for(Translation2d circle : circlePoints) {
        //     Point scaledPoint = new Point( (WIDTH/2) + (circle.getX() / scale), (WIDTH/2) + (circle.getY() / scale));
        //     Imgproc.circle(mat, scaledPoint, (int) (RADIUS_OFFSET / scale), new Scalar(0,255,255));
        //     // System.out.println(circle.x + " - " + circle.y);
        // }
        
    

        closestBall = new Translation2d();

        if (circlePoints.isEmpty()) {
            closestBall = new Translation2d(Double.NaN, Double.NaN);
        } else {
            double minDist = Double.POSITIVE_INFINITY;
            Translation2d best = null;
            for (Translation2d circle : circlePoints) {
                double dist = circle.getSquaredNorm(); // distance from 0,0
                if (dist < minDist) {
                        minDist = dist;
                        best = circle;
                    }
                }

            closestBall = best;
        }

        if (closestBallPrior != null) {
            if (closestBall.getDistance(closestBallPrior) < 0.1 && outOfBounds(closestBall)){

                // Point scaledPoint = new Point( (WIDTH/2) + (closestBall.getX() / scale), (WIDTH/2) + (closestBall.getY() / scale));
                // Imgproc.circle(mat, scaledPoint, (int) (RADIUS_OFFSET / scale), new Scalar(200, 25, 52), -1);
                latestBallAngleDeg = new Rotation2d(Math.atan((closestBall.getY())/(closestBall.getX()))/Math.PI*180);
            } else {
                // Point scaledPoint = new Point( (WIDTH/2) + (closestBallPrior.getX() / scale), (WIDTH/2) + (closestBallPrior.getY() / scale));
                // Imgproc.circle(mat, scaledPoint, (int) (RADIUS_OFFSET / scale), new Scalar(200, 25, 52), -1);
            }
        }

        closestBallPrior = closestBall;



        // Imgproc.circle(mat, LIDAR, (int) (RADIUS_OFFSET / scale), new Scalar(255,255,255), -1);
        
        // System.o
        

        // showWindow(mat);
    }

    private static void showWindow(Mat img) {
        // Display the image in a window titled "Original Image"
        HighGui.imshow("Original Image", img);


        // Wait for a key press to close the window
        HighGui.waitKey(1);
    }

    // XYZ Position of the LiDAR on the robot 
    private static final Translation2d LiDAR_POS = new Translation2d(1, 0);

    // Angle of the lidar unit 
    private static final double LiDAR_PITCH = 0; // Radians
    private static final double LiDAR_ROLL = 0; // Radians

    // Convert a LiDAR ball position to a field position
    public static Translation2d lidarPosToField(Translation2d p, Pose2d pose) {
        // Project the point tilted plane on to the XY plane
        // Point should be relative to the XY plane, with (0,0) centered at the centerpoint of the lidar
        double x = p.getX() * Math.cos(LiDAR_ROLL) + p.getY() * Math.sin(LiDAR_PITCH) * Math.sin(LiDAR_ROLL);
        double y = p.getY() * Math.cos(LiDAR_PITCH);

        // Translate the ball position to relative to the center of the robot
        // Point should be relative to robot, wth (0,0) centered at center of robot
        x -= LiDAR_POS.getX();
        y -= LiDAR_POS.getY();

        // Rotate the point by the robot's rotation
        // Point should now be relative to robot, but rotated relative to the field.
        double ang = -pose.getRotation().getRadians();
        x = x*Math.cos(ang) - y*Math.sin(ang);
        y = x*Math.sin(ang) + y*Math.cos(ang);

        // Translate the point to the robot's field position
        // Point should be relative to field. (0,0) should be relative to the field.
        x += pose.getX();
        y += pose.getY();

        return new Translation2d(x, y);
    }
}