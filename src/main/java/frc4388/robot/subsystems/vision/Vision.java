package frc4388.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.subsystems.vision.VisionIO.PoseObservation;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;
import frc4388.utility.status.Status;

public class Vision extends SubsystemBase implements Queryable {
    VisionIO[] io;
    VisionStateAutoLogged[] state;

    
    public Pose2d lastVisionPose = new Pose2d();
    public Pose2d lastPhysOdomPose = new Pose2d();

    public Vision(VisionIO... devices) {
        FaultReporter.register(this);
        io = devices;
        state = new VisionStateAutoLogged[io.length];

        for(int i = 0; i < io.length; i++) {
            state[i] = new VisionStateAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < io.length; i++) {
            io[i].updateInputs(state[i]);
            Logger.processInputs("Vision/Camera" + i , state[i]);
        }
    }

    public List<PoseObservation> getPosesToAdd(){
        List<PoseObservation> poses = new ArrayList<>();
        for(int i = 0; i < state.length; i++) {
            if(state[i].lastEstimatedPose != null) {
                poses.add(state[i].lastEstimatedPose);
            }
        }

        return poses;
    }

    public void setLastOdomPose(Pose2d pose){
        if(pose != null)
            lastPhysOdomPose = pose;
    }

    public boolean isTag(){
        for(int i = 0; i < state.length; i++){
            if(state[i].isTagDetected && state[i].isTagProcessed)
                return true;
        }
        return false;
    }

    @AutoLogOutput
    public Pose2d getPose2d() {
        if(lastPhysOdomPose != null)
            return lastPhysOdomPose;

        // if(lastVisionPose != null)
        //     return lastVisionPose;
        return new Pose2d();

    }

    public static double getTime() {
        return Utils.getCurrentTimeSeconds();
    }


    @Override
    public Status diagnosticStatus() {
        return new Status();
        // // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'diagnosticStatus'");
    }
    
}
