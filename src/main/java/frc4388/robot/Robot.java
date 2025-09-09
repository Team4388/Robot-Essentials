/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

// Advantagekit
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4388.robot.constants.BuildConstants;
import frc4388.robot.constants.Constants.SimConstants;
import frc4388.utility.DeferredBlock;
import frc4388.utility.compute.RobotTime;
import frc4388.utility.compute.Trim;
import frc4388.utility.status.FaultReporter;

//import frc4388.robot.subsystems.LED;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  Command m_autonomousCommand;

  private RobotTime m_robotTime = RobotTime.getInstance();
  private RobotContainer m_robotContainer;
  //private LED mled = new LED();
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Start logging with AdvantageKit
    startLogging();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


    // // Create a shuffleboard update thread, that will periodically update the values on shuffleboard
    // FaultReporter.startThread();
  }



  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.doubl
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {   
    m_robotTime.updateTimes();
    // SmartDashboard.putNumber("Time", System.currentTimeMillis());
    
    m_robotContainer.m_robotLED.update();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }
  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_robotTime.endMatchTime();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    DeferredBlock.execute();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotTime.startMatchTime();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }


  @Override
  public void teleopInit() {
    m_robotContainer.stop();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
      m_autonomousCommand.cancel();
      m_autonomousCommand.end(true);

    }
    m_robotTime.startMatchTime();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  //  m_robotContainer.m_robotMap.rightFront.go(m_robotContainer.getDeadbandedDriverController().getLeft());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopExit() { // the only OTHER mode that teleop can enter into is disabled.
    Trim.dumpAll();
  }

  @Override
  public void testInit() {
    FaultReporter.printReport();
  }







  // VisionSystemSim visionSim;
  // RobotMapSim robotMapSim;

  // @Override
  // public void simulationInit() {
  //   visionSim = new VisionSystemSim("main");
  //   robotMapSim = m_robotContainer.m_robotMap.configureSim();


  //   // A 0.5 x 0.25 meter rectangular target
  //   TargetModel targetModel = new TargetModel(0.5, 0.25);
  //   // The pose of where the target is on the field.
  //   // Its rotation determines where "forward" or the target x-axis points.
  //   // Let's say this target is flat against the far wall center, facing the blue driver stations.
  //   Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  //   // The given target model at the given pose
  //   VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

  //   // Add this vision target to the vision system simulation to make it visible
  //   visionSim.addVisionTargets(visionTarget);

  //   // The layout of AprilTags which we want to add to the vision system
  //   AprilTagFieldLayout tagLayout = Constants.FieldConstants.kTagLayout;

  //   visionSim.addAprilTags(tagLayout);


  //   visionSim.addCamera(robotMapSim.leftCamera, Constants.VisionConstants.LEFT_CAMERA_POS);
  //   visionSim.addCamera(robotMapSim.rightCamera, Constants.VisionConstants.RIGHT_CAMERA_POS);

  //   SmartDashboard.putData(visionSim.getDebugField());
  // }


  // @Override
  // public void simulationPeriodic() {
  //   m_robotContainer.m_robotSwerveDrive.updateSim(RobotController.getBatteryVoltage());
  //   // visionSim.update(m_robotContainer.m_robotSwerveDrive.getPose2d());

  //   // m_robotContainer.m_robotSwerveDrive.
  // }



// Start AdvantageKit logging / replay and record metadata
// Refrence: https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/skeleton/src/main/java/frc/robot/Robot.java
  public void startLogging() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("Git+SHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (SimConstants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }
}