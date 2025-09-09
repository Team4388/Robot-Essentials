/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc4388.robot.constants.Constants.ElevatorConstants;
import frc4388.robot.constants.Constants.LiDARConstants;
import frc4388.robot.constants.Constants.SimConstants;
import frc4388.robot.constants.Constants.VisionConstants;
import frc4388.robot.subsystems.elevator.ElevatorIO;
import frc4388.robot.subsystems.elevator.ElevatorReal;
import frc4388.robot.subsystems.lidar.LiDAR;
import frc4388.robot.subsystems.lidar.LidarIO;
import frc4388.robot.subsystems.lidar.LidarReal;
import frc4388.robot.subsystems.swerve.SwerveDriveConstants;
import frc4388.robot.subsystems.swerve.SwerveIO;
import frc4388.robot.subsystems.swerve.SwerveReal;
import frc4388.robot.subsystems.vision.VisionIO;
import frc4388.robot.subsystems.vision.VisionReal;
import frc4388.utility.status.FaultCANCoder;
import frc4388.utility.status.FaultPhotonCamera;
import frc4388.utility.status.FaultPidgeon2;
import frc4388.utility.status.FaultTalonFX;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
    // private Pigeon2 m_pigeon2 = new Pigeon2(SwerveDriveConstants.IDs.DRIVE_PIGEON.id);
    // public RobotGyro gyro = new RobotGyro(m_pigeon2);

    public final VisionIO leftCamera;
    public final VisionIO rightCamera;

    // public final LiDAR lidar = new 

    public final LidarIO reefLidar;
    public final LidarIO reverseLidar;


    /* LED Subsystem */
    // public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);
    
    /* Swreve Drive Subsystem */
    public final SwerveIO swerveDrivetrain;

    /* Elevator Subsystem */
    public final ElevatorIO elevatorIO;

    public RobotMap(SimConstants.Mode mode) {
        switch (mode) {
            case REAL:
                // Configure cameras
                PhotonCamera leftCameraReal = new PhotonCamera(VisionConstants.LEFT_CAMERA_NAME);
                PhotonCamera rightCameraReal = new PhotonCamera(VisionConstants.RIGHT_CAMERA_NAME);

                leftCamera =  new VisionReal(leftCameraReal, VisionConstants.LEFT_CAMERA_POS);                ;
                rightCamera = new VisionReal(rightCameraReal, VisionConstants.RIGHT_CAMERA_POS);

                FaultPhotonCamera.addDevice(leftCameraReal, "Left Camera");
                FaultPhotonCamera.addDevice(rightCameraReal , "Right Camera");

                // Configure LiDAR
                reefLidar = new LidarReal(LiDARConstants.REEF_LIDAR_DIO_CHANNEL);
                reverseLidar = new LidarReal(LiDARConstants.REVERSE_LIDAR_DIO_CHANNEL);

                // Configure swerve drive train
                SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDrivetrainReal = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder> (TalonFX::new, TalonFX::new, CANcoder::new, 
                    SwerveDriveConstants.DrivetrainConstants, 
                    SwerveDriveConstants.FRONT_LEFT, SwerveDriveConstants.FRONT_RIGHT,
                    SwerveDriveConstants.BACK_LEFT, SwerveDriveConstants.BACK_RIGHT
                );

                swerveDrivetrain = new SwerveReal(swerveDrivetrainReal);

                // Configure elevator

                TalonFX elevator = new TalonFX(ElevatorConstants.ELEVATOR_ID.id);
                TalonFX endeffector = new TalonFX(ElevatorConstants.ENDEFFECTOR_ID.id);
                

                DigitalInput basinLimitSwitch = new DigitalInput(ElevatorConstants.BASIN_LIMIT_SWITCH);
                DigitalInput endeffectorLimitSwitch = new DigitalInput(ElevatorConstants.ENDEFFECTOR_LIMIT_SWITCH);
                DigitalInput IRIntakeBeam = new DigitalInput(ElevatorConstants.INTAKE_LIMIT_SWITCH);

                elevatorIO = new ElevatorReal(elevator, endeffector, basinLimitSwitch, endeffectorLimitSwitch, IRIntakeBeam);



                // Fault
                FaultPidgeon2.addDevice(swerveDrivetrainReal.getPigeon2(), "Gyro");

                FaultTalonFX.addDevice(elevator, "Elevator");
                FaultTalonFX.addDevice(endeffector, "Endeffector");
                
                FaultTalonFX.addDevice(swerveDrivetrainReal.getModule(0).getDriveMotor(), "Module 0 Drive");
                FaultTalonFX.addDevice(swerveDrivetrainReal.getModule(0).getSteerMotor(), "Module 0 Steer");
                FaultCANCoder.addDevice(swerveDrivetrainReal.getModule(0).getEncoder(), "Module 0 CANCoder");
                FaultTalonFX.addDevice(swerveDrivetrainReal.getModule(1).getDriveMotor(), "Module 1 Drive");
                FaultTalonFX.addDevice(swerveDrivetrainReal.getModule(1).getSteerMotor(), "Module 1 Steer");
                FaultCANCoder.addDevice(swerveDrivetrainReal.getModule(1).getEncoder(), "Module 1 CANCoder");
                FaultTalonFX.addDevice(swerveDrivetrainReal.getModule(2).getDriveMotor(), "Module 2 Drive");
                FaultTalonFX.addDevice(swerveDrivetrainReal.getModule(2).getSteerMotor(), "Module 2 Steer");
                FaultCANCoder.addDevice(swerveDrivetrainReal.getModule(2).getEncoder(), "Module 2 CANCoder");
                FaultTalonFX.addDevice(swerveDrivetrainReal.getModule(3).getDriveMotor(), "Module 3 Drive");
                FaultTalonFX.addDevice(swerveDrivetrainReal.getModule(3).getSteerMotor(), "Module 3 Steer");
                FaultCANCoder.addDevice(swerveDrivetrainReal.getModule(3).getEncoder(), "Module 3 CANCoder");

                break;
            // case SIM:
            //     break;
            default:
                leftCamera = new VisionIO() {};
                rightCamera = new VisionIO() {};
                reefLidar = new LidarIO() {};
                reverseLidar = new LidarIO() {};
                swerveDrivetrain = new SwerveIO() {};
                elevatorIO = new ElevatorIO() {};
                break;
        }
    }

    // public class RobotMapSim {
    //     public PhotonCameraSim leftCamera;
    //     public PhotonCameraSim rightCamera;
    // }

    // public RobotMapSim configureSim() {
    //     RobotMapSim sim = new RobotMapSim();

    //     // The simulated camera properties
    //     SimCameraProperties cameraProp = new SimCameraProperties();
    //     // A 640 x 480 camera with a 100 degree diagonal FOV.
    //     cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    //     // Approximate detection noise with average and standard deviation error in pixels.
    //     cameraProp.setCalibError(0.25, 0.08);
    //     // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    //     cameraProp.setFPS(20);
    //     // The average and standard deviation in milliseconds of image data latency.
    //     cameraProp.setAvgLatencyMs(35);
    //     cameraProp.setLatencyStdDevMs(5);

    //     // sim.leftCamera = new PhotonCameraSim(leftCamera, cameraProp);
    //     // sim.rightCamera = new PhotonCameraSim(rightCamera, cameraProp);

        
    //     sim.leftCamera.enableRawStream(true);
    //     sim.leftCamera.enableProcessedStream(true);
    //     sim.leftCamera.enableDrawWireframe(true);


    //     sim.rightCamera.enableRawStream(true);
    //     sim.rightCamera.enableProcessedStream(true);
    //     sim.rightCamera.enableDrawWireframe(true);

    //     return sim;

    // }
   
}