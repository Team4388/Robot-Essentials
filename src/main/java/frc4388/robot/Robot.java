/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.Base64;
import java.util.List;
import java.util.logging.Level;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4388.utility.CanDevice;
import frc4388.utility.DeferredBlock;
import frc4388.utility.RobotTime;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;
import frc4388.utility.DeviceFinder;
import frc4388.utility.Status.Report;
//import frc4388.robot.subsystems.LED;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
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

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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
    //System.out.println(m_robotContainer.limelight.isNearSpeaker());
    //mled.updateLED();
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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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

  @Override
  public void testInit() {

    // Subsystems header
    System.out.println(new String(Base64.getDecoder().decode("IF9fICAgICAgIF8gICAgICAgICAgICAgICAgICAgXyAgICAgICAgICAgICAgICAgICAgIAovIF9cXyAgIF98IHxfXyAgX19fIF8gICBfIF9fX3wgfF8gX19fIF8gX18gX19fICBfX18gClwgXHwgfCB8IHwgJ18gXC8gX198IHwgfCAvIF9ffCBfXy8gXyBcICdfIGAgXyBcLyBfX3wKX1wgXCB8X3wgfCB8XykgXF9fIFwgfF98IFxfXyBcIHx8ICBfXy8gfCB8IHwgfCBcX18gXApcX18vXF9fLF98XyBfXy98X19fL1xfXywgfF9fXy9cX19cX19ffF98IHxffCB8X3xfX18vCiAgICAgICAgICAgICAgICAgICAgfF9fXy8gICAgICAgICAgICAgICAgICAgICAgICAgICA=")));

    for(int i=0;i<m_robotContainer.subsystems.size();i++){

      Subsystem subsystem = m_robotContainer.subsystems.get(i);
      System.out.println("** Subsystem diagnostic report for " + subsystem.getName() + ":");
      Status status = subsystem.diagnosticStatus();

      for(int a=0;a<status.reports.size();a++){
        Report r = status.reports.get(a);
        subsystem.Log(r.toString());
      }
    }

    
    // CAN header
    System.out.println(new String(Base64.getDecoder().decode("ICAgX19fICAgXyAgICAgICAgX18gCiAgLyBfX1wgL19cICAgIC9cIFwgXAogLyAvICAgLy9fXFwgIC8gIFwvIC8KLyAvX19fLyAgXyAgXC8gL1wgIC8gIApcX19fXy9cXy8gXF8vXF9cIFwvICh0KQ==")));
    

    for(int i=0;i<CanDevice.devices.size();i++){

      CanDevice device = CanDevice.devices.get(i);
      System.out.println("** CAN diagnostic report for " + device.name + ":");
      Status status = device.diagnosticStatus();

      for(int a=0;a<status.reports.size();a++){
        Report r = status.reports.get(a);
        device.Log(r.toString());
      }
    }

    System.out.println("Found CAN devices: " + new DeviceFinder().Find());
  }
}
