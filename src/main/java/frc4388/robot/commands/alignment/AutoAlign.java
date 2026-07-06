package frc4388.robot.commands.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.constants.Constants.AutoConstants;
import frc4388.robot.subsystems.swerve.SwerveDrive;
import frc4388.robot.subsystems.vision.Lidar;
import frc4388.robot.subsystems.vision.Vision;
import frc4388.utility.compute.FieldPositions;
import frc4388.utility.structs.Gains;

public class AutoAlign extends Command {

    private PID rotPID = new PID(AutoConstants.ROT_GAINS, 0);
    private Lidar lidar;
    private boolean isLidar;

    private Pose2d targetpos;
    private double targetRotation;

    SwerveDrive swerveDrive;
    Vision vision;

    public AutoAlign(SwerveDrive swerveDrive, Vision vision, Lidar lidar, boolean isLidar) {
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.lidar = lidar;
        this.isLidar = isLidar;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        rotPID.initialize();
        this.targetRotation = swerveDrive.getPose2d().getRotation().getDegrees();
        //this.targetpos = new Pose2d(FieldPositions.HUB_POSITION, new Rotation2d(0));
    }
    

    double roterr;

    double rotoutput;

    @Override
    public void execute() {
        double desiredHeading;
        // Pose2d robotPose = vision.getPose2d();
        targetpos = new Pose2d(lidar.getClosestBall(), new Rotation2d(0));
        // if (robotPose == null) return;
        if (targetpos == null) return;
        if (targetpos.getTranslation() == null) return;


        double dx = targetpos.getX();// - robotPose.getX();
        double dy = targetpos.getY();// - robotPose.getY();

        if (!isLidar){
            desiredHeading = (Math.atan2(dy, dx)+ Math.PI) * (180. / Math.PI) + 180;
        }else{
            desiredHeading = (Math.atan2(dx, dy)) * (180. / Math.PI);// + 180;
        }


        targetRotation = swerveDrive.getPose2d().getRotation().getDegrees() - desiredHeading;

        // roterr = desiredHeading.getDegrees() - robotPose.getRotation().getDegrees();
        // if (roterr > 180) roterr -= 360;
        // if (roterr < -180) roterr += 360;

        SmartDashboard.putNumber("Target Rotation!", targetRotation);
        // System.out.println("Target: " + targetpos + "Heading: " + desiredHeading + "Error: " + roterr);
        swerveDrive.driveRelativeAngle(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(targetRotation));
    }

    @Override
    public final boolean isFinished() {
        // boolean finished = Math.abs(roterr) < AutoConstants.ROT_TOLERANCE;
        // if (finished) {
        //     swerveDrive.softStop();
        // }
        // return finished;
        return false;
    }

    private class PID {
        protected Gains  gains;
        private   double output    = 0;


        /** Creates a new PelvicInflammatoryDisease. */
        public PID(Gains gains, double tolerance) {
            this.gains     = gains;
        }

        // Called when the command is initially scheduled.
        public final void initialize() {
            output = 0;
        }

        private double prevError, cumError = 0;
        
        // Called every time the scheduler runs while the command is scheduled.
        public double update(double error) {
            cumError += error * .02; // 20 ms
            double delta = error - prevError;

            output = error * gains.kP;
            output += cumError * gains.kI;
            output += delta * gains.kD;
            output += gains.kF;

            return output;
        }
    }
    
}
