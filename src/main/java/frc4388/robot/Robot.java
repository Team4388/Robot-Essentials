/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.ArrayList;
import java.util.Base64;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4388.utility.CanDevice;
import frc4388.utility.DeferredBlock;
import frc4388.utility.DeferredBlockMulti;
import frc4388.utility.RobotTime;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;
import frc4388.utility.Trim;
import frc4388.utility.Status.Report;
import frc4388.utility.Status.ReportLevel;
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



    new Thread() {
      public void run() {
        try{
        while(!this.isInterrupted() && this.isAlive()){
          Thread.sleep(500);
          for(int i=0;i<Subsystem.subsystems.size(); i++){
            Subsystem.subsystems.get(i).queryStatus();
          }

          // System.out.println("Updated statuses!");
          
        }
        }catch(Exception e){
          e.printStackTrace();
        }
      }
    }.start();
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
    DeferredBlockMulti.execute();
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
      System.out.println("NOT Null!!");

    } else {
      System.out.println("Null!!");
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

    List<String> errors = new ArrayList<>();

    // Subsystems header
    System.out.println(new String(Base64.getDecoder().decode("IOKWl+KWhOKWhOKWluKWl+KWliDilpfilpbilpfiloTiloTilpYgIOKWl+KWhOKWhOKWluKWl+KWliAg4paX4paW4paX4paE4paE4paW4paX4paE4paE4paE4paW4paX4paE4paE4paE4paW4paX4paWICDilpfilpYg4paX4paE4paE4paWCuKWkOKWjCAgIOKWkOKWjCDilpDilozilpDilowg4paQ4paM4paQ4paMICAgIOKWneKWmuKWnuKWmOKWkOKWjCAgICAg4paIICDilpDilowgICDilpDilpvilprilp7ilpzilozilpDilowgICAKIOKWneKWgOKWmuKWluKWkOKWjCDilpDilozilpDilpviloDilprilpYg4pad4paA4paa4paWICDilpDilowgIOKWneKWgOKWmuKWliAg4paIICDilpDilpviloDiloDilpjilpDilowgIOKWkOKWjCDilp3iloDilprilpYK4paX4paE4paE4pae4paY4pad4paa4paE4pae4paY4paQ4paZ4paE4pae4paY4paX4paE4paE4pae4paYICDilpDilowg4paX4paE4paE4pae4paYICDiloggIOKWkOKWmeKWhOKWhOKWluKWkOKWjCAg4paQ4paM4paX4paE4paE4pae4paY")));

    for(int i=0;i< Subsystem.subsystems.size();i++){

      Subsystem subsystem = Subsystem.subsystems.get(i);
      System.out.println("** Subsystem diagnostic report for " + subsystem.getName() + ":");
      Status status = subsystem.diagnosticStatus();

      for(int a=0;a<status.reports.size();a++){
        Report r = status.reports.get(a);
        if(r.reportLevel == ReportLevel.ERROR)
          errors.add(subsystem.getName() + " - " + r.toString());
        subsystem.Log(r.toString());
      }
    }

    
    // CAN header
    System.out.println(new String(Base64.getDecoder().decode("IOKWl+KWhOKWhOKWliDilpfiloTilpYg4paX4paWICDilpfilpYK4paQ4paMICAg4paQ4paMIOKWkOKWjOKWkOKWm+KWmuKWluKWkOKWjArilpDilowgICDilpDilpviloDilpzilozilpDilowg4pad4pac4paMCuKWneKWmuKWhOKWhOKWluKWkOKWjCDilpDilozilpDilowgIOKWkOKWjCh0KQ==")));
    
    CANBus canBus = new CANBus(Constants.CANBUS_NAME);
    
    CANBusStatus canInfo = canBus.getStatus();
    
    System.out.println("CANInfo BusOffCount     - " + canInfo.BusOffCount);
    System.out.println("CANInfo BusUtilization  - " + canInfo.BusUtilization);
    System.out.println("CANInfo RX Errors count - " + canInfo.REC);
    System.out.println("CANInfo TX Errors count - " + canInfo.TEC);
    System.out.println("CANInfo Transmit buffer full count - " + canInfo.TxFullCount);
    // Broken turniary operator
    ReportLevel canReportLevel = canInfo.Status.isOK() ? (canInfo.Status.isWarning() ? ReportLevel.WARNING : ReportLevel.ERROR) : ReportLevel.INFO;
    String canStatus = "CAN " + canReportLevel.name() + " - " + canInfo.Status.getName() + " (" + canInfo.Status.getDescription() + ")";
    if(canReportLevel == ReportLevel.ERROR) {
      errors.add(canStatus);
    }
    System.out.println(canStatus);

    for(int i=0;i<CanDevice.devices.size();i++){

      CanDevice device = CanDevice.devices.get(i);
      System.out.println("** CAN diagnostic report for " + device.name + ":");
      Status status = device.diagnosticStatus();

      for(int a=0;a<status.reports.size();a++){
        Report r = status.reports.get(a);
        if(r.reportLevel == ReportLevel.ERROR)
          errors.add(device.getName() + " - " + r.toString());
        device.Log(r.toString());
      }
    }

    // System.out.println("Found CAN devices: " + new DeviceFinder().Find());
    
    if(errors.size() > 0) {
      // Errors header
      System.out.println(new String(Base64.getDecoder().decode("4paX4paE4paE4paE4paW4paX4paE4paE4paWIOKWl+KWhOKWhOKWliAg4paX4paE4paWIOKWl+KWhOKWhOKWliAg4paX4paE4paE4paWCuKWkOKWjCAgIOKWkOKWjCDilpDilozilpDilowg4paQ4paM4paQ4paMIOKWkOKWjOKWkOKWjCDilpDilozilpDilowgICAK4paQ4pab4paA4paA4paY4paQ4pab4paA4paa4paW4paQ4pab4paA4paa4paW4paQ4paMIOKWkOKWjOKWkOKWm+KWgOKWmuKWliDilp3iloDilprilpYK4paQ4paZ4paE4paE4paW4paQ4paMIOKWkOKWjOKWkOKWjCDilpDilozilp3ilpriloTilp7ilpjilpDilowg4paQ4paM4paX4paE4paE4pae4paY")));
      for(int i=0;i<errors.size(); i++){
        System.out.println(errors.get(i));
      }
    }

  }
}
