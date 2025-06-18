/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import frc4388.robot.Constants.ElevatorConstants;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.Constants.VisionConstants;
// import frc4388.robot.subsystems.SwerveModule;
import frc4388.utility.RobotGyro;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
    // private Pigeon2 m_pigeon2 = new Pigeon2(SwerveDriveConstants.IDs.DRIVE_PIGEON.id);
    // public RobotGyro gyro = new RobotGyro(m_pigeon2);

    public PhotonCamera leftCamera = new PhotonCamera(VisionConstants.LEFT_CAMERA_NAME);
    public PhotonCamera rightCamera = new PhotonCamera(VisionConstants.RIGHT_CAMERA_NAME);
    
    public RobotMap() {
        configureDriveMotorControllers();
    }

    /* LED Subsystem */
    // public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);
    
    /* Swreve Drive Subsystem */
    public final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDrivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder> (TalonFX::new, TalonFX::new, CANcoder::new, 
        Constants.SwerveDriveConstants.DrivetrainConstants, 
        Constants.SwerveDriveConstants.FRONT_LEFT, Constants.SwerveDriveConstants.FRONT_RIGHT,
        Constants.SwerveDriveConstants.BACK_LEFT, Constants.SwerveDriveConstants.BACK_RIGHT
    );

    /* Elevator Subsystem */
    public final TalonFX elevator = new TalonFX(ElevatorConstants.ELEVATOR_ID.id);
    public final TalonFX endeffector = new TalonFX(ElevatorConstants.ENDEFFECTOR_ID.id);
    

    public final DigitalInput basinLimitSwitch = new DigitalInput(ElevatorConstants.BASIN_LIMIT_SWITCH);
    public final DigitalInput endeffectorLimitSwitch = new DigitalInput(ElevatorConstants.ENDEFFECTOR_LIMIT_SWITCH);
    public final DigitalInput IRIntakeBeam = new DigitalInput(ElevatorConstants.INTAKE_LIMIT_SWITCH);

    void configureDriveMotorControllers() {
        // endeffector.saf
    }
        
}