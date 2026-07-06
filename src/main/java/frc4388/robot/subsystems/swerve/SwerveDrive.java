// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants.AutoConstants;
import frc4388.robot.subsystems.vision.Vision;
import frc4388.utility.compute.TimesNegativeOne;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;
import frc4388.utility.status.Status;

public class SwerveDrive extends SubsystemBase implements Queryable {
    // private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;

    private SwerveIO io;
    private SwerveStateAutoLogged state;

    private Vision vision;



    public int gear_index = SwerveDriveConstants.STARTING_GEAR;
    public boolean stopped = false;
    public boolean robotKnowsWhereItIs = false;

    public double speedAdjust = SwerveDriveConstants.MAX_SPEED_MEETERS_PER_SEC * SwerveDriveConstants.GEARS[gear_index];
    public double rotSpeedAdjust = SwerveDriveConstants.MAX_ROT_SPEED;
    public double autoSpeedAdjust = SwerveDriveConstants.MAX_SPEED_MEETERS_PER_SEC * 0.25; // cap auto performance to
                                                                                            // 25%

    public double lastOdomSpeed;

    public Pose2d initalPose2d = new Pose2d();



    public double rotTarget = 0.0;
    public Rotation2d orientRotTarget = new Rotation2d();
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private final PIDController m_rotationOverridePID = new PIDController(
        SwerveDriveConstants.PIDConstants.AIM_kP.get(),
        SwerveDriveConstants.PIDConstants.AIM_kI.get(),
        SwerveDriveConstants.PIDConstants.AIM_kD.get()
    );
    private boolean m_useRotationOverride = false;
    private Translation2d m_rotationOverrideTarget = new Translation2d();

    /** Creates a new SwerveDrive. */
    public SwerveDrive(SwerveIO swerveDriveTrain, Vision vision) {
        // public SwerveDrive(SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        // swerveDriveTrain) {
        FaultReporter.register(this);

        this.io = swerveDriveTrain;
        this.state = new SwerveStateAutoLogged();

        this.vision = vision;

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            config = null;
        }

        PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID
            new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID (used when override is OFF)
        );
        driveController.setRotationTargetOverride(() -> {
            if (!m_useRotationOverride) return Optional.empty();
            Rotation2d targetAngle = getPose2d()
                .getTranslation()
                .minus(m_rotationOverrideTarget)
                .getAngle();
            return Optional.of(targetAngle);
        });

        // DoubleSupplier a = () -> 1.d;
        AutoBuilder.configure(
                () -> {
                    return getPose2d();
                }, // Robot pose supplier
                this::setOdoPose, // Method to reset odometry (will be called if your auto has a starting
                                             // pose)
                () -> state.speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> io.setControl(new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
                                              // Also optionally outputs individual module feedforwards
                driveController, // <-- use the variable, not inline new PPHolonomicDriveController(...)
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Red;
                    // }
                    return TimesNegativeOne.isRed;
                },
                this // Reference to this subsystem to set requirements
        );

        PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        
        PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });



        // // Configure SysId
        // sysId =
        //     new SysIdRoutine(
        //         new SysIdRoutine.Config(
        //             null,
        //             null,
        //             null,
        //             (state) -> Logger.recordOutput("Drive/SysIdState", toString())),
        //         new SysIdRoutine.Mechanism(
        //             (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    }

    public void setOdoPose(Pose2d pose) {
        if (pose == null) return;
        initalPose2d = pose;
        io.resetPose(initalPose2d);
    }

    public void setInitalPose(Pose2d startingAutoPose){
        initalPose2d = startingAutoPose;
    }


    // public void oneModuleTest(SwerveModule module, Translation2d leftStick,
    // Translation2d rightStick){
    // // double ang = Math.atan2(rightStick.getY(), rightStick.getX());
    // // rightStick.getAngle()
    // double speed = Math.sqrt(Math.pow(leftStick.getX(), 2) +
    // Math.pow(leftStick.getY(), 2));
    // // System.out.println(ang);
    // // module.go(ang);
    // // Rotation2d rot = Rotation2d.fromRadians(ang);
    // Rotation2d rot = new Rotation2d(rightStick.getX(), rightStick.getY());
    // SwerveModuleState state = new SwerveModuleState(speed, rot);
    // module.setDesiredState(state);
    // }

    public double chassisXSpeeds(){
        if (TimesNegativeOne.isRed) {
            return chassisSpeeds.vxMetersPerSecond;
        } else {
            return -chassisSpeeds.vxMetersPerSecond;
        }
    }

    public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
        if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05 && stopped == false) // if no imput and the swerve drive is still going:
            stopModules(); // stop the swerve

        if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05) // if no imput
            return; // don't bother doing swerve drive math and return early.

        leftStick = leftStick.rotateBy(TimesNegativeOne.ForwardOffset);
        
        stopped = false;
        if (fieldRelative) {
            
            leftStick = TimesNegativeOne.invert(leftStick, TimesNegativeOne.XAxis, TimesNegativeOne.YAxis);
            rightStick = TimesNegativeOne.invert(rightStick, TimesNegativeOne.RotAxis);    

            // ! drift correction
            if (rightStick.getNorm() > 0.05 || !SwerveDriveConstants.DRIFT_CORRECTION_ENABLED) {

                rotTarget = state.currentPose.getRotation().getDegrees();

                io.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(leftStick.getX() * speedAdjust)
                    .withVelocityY(leftStick.getY() * speedAdjust)
                    .withRotationalRate(rightStick.getX() * rotSpeedAdjust));

                    // .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
                SmartDashboard.putBoolean("drift correction", false);
            } else {
                var ctrl = new SwerveRequest.FieldCentricFacingAngle()
                    .withVelocityX(leftStick.getX() * speedAdjust)
                    .withVelocityY(leftStick.getY() * speedAdjust)
                    .withTargetDirection(Rotation2d.fromDegrees(rotTarget));
                ctrl.HeadingController.setPID(
                    SwerveDriveConstants.PIDConstants.DRIFT_CORRECTION_GAINS.kP,
                    SwerveDriveConstants.PIDConstants.DRIFT_CORRECTION_GAINS.kI,
                    SwerveDriveConstants.PIDConstants.DRIFT_CORRECTION_GAINS.kD
                );
                io.setControl(ctrl);
                SmartDashboard.putBoolean("drift correction", true);
            }

           
        } else { // Create robot-relative speeds.
            io.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(leftStick.getX() * speedAdjust)
                    .withVelocityY(-leftStick.getY() * speedAdjust)
                    .withRotationalRate(rightStick.getX() * rotSpeedAdjust));
        }
    }

    public void driveFine(Translation2d leftStick, Translation2d rightStick, double percentOutput) {
        stopped = false;
        // Create robot-relative speeds.
        if (rightStick.getNorm() > 0.1) rightStick = rightStick.times(0);
        io.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(leftStick.getX() * SwerveDriveConstants.MAX_SPEED_MEETERS_PER_SEC * percentOutput)
            .withVelocityY(-leftStick.getY() * SwerveDriveConstants.MAX_SPEED_MEETERS_PER_SEC * percentOutput)
            .withRotationalRate(rightStick.getX() * rotSpeedAdjust));
        
    }

    

    public void aimAtPosition(Translation2d fieldPos, double aimLeadTime) {
        Translation2d robotSpeed = new Translation2d(
            chassisSpeeds.vxMetersPerSecond, 
            chassisSpeeds.vyMetersPerSecond
        );
        Translation2d fieldPosLead = robotSpeed.times(aimLeadTime).plus(fieldPos);
        Rotation2d ang = getPose2d().getTranslation().minus(fieldPosLead).getAngle();

        var ctrl = new SwerveRequest.FieldCentricFacingAngle()
        .withVelocityX(chassisSpeeds.vxMetersPerSecond)
        .withVelocityY(chassisSpeeds.vyMetersPerSecond)
        .withTargetDirection(ang);
        ctrl.HeadingController.setPID(
        SwerveDriveConstants.PIDConstants.AIM_kP.get(),
        SwerveDriveConstants.PIDConstants.AIM_kI.get(),
        SwerveDriveConstants.PIDConstants.AIM_kD.get()
        );
        io.setControl(ctrl);
    }

    public void driveWithInputOrientation(Translation2d leftStick, Translation2d rightStick) { // there is no practical
                                                                                               // reason to have a robot
                                                                                               // relitive version of
                                                                                               // this, and no pre
                                                                                               // provided version
        if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05 && stopped == false) // if no imput and the swerve
                                                                                           // drive is still going:
            stopModules(); // stop the swerve

        if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05) // if no imput
            return; // don't bother doing swerve drive math and return early.

        leftStick.rotateBy(TimesNegativeOne.ForwardOffset);

        if(!TimesNegativeOne.isRed) {
            leftStick.rotateBy(new Rotation2d(Math.PI/2.));
        }

        io.setControl(new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(leftStick.getX() * speedAdjust)
                .withVelocityY(leftStick.getY() * speedAdjust)
                .withTargetDirection(rightStick.getAngle()));
    }

    public void driveRelativeAngle(Translation2d leftStick, Rotation2d heading) {
        
        leftStick = leftStick.rotateBy(TimesNegativeOne.ForwardOffset);
        leftStick = TimesNegativeOne.invert(leftStick, TimesNegativeOne.XAxis, TimesNegativeOne.YAxis);
        var ctrl = new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(leftStick.getX() * speedAdjust)
            .withVelocityY(leftStick.getY() * speedAdjust)
            .withTargetDirection(heading);
        ctrl.HeadingController.setPID(
            SwerveDriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kP,
            SwerveDriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kI,
            SwerveDriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kD
        );
        io.setControl(ctrl);
    }

    // Drive with a specific velocity and heading
    public void driveFieldAngle(Translation2d leftStick, Rotation2d heading) {
        if (leftStick.getNorm() < 0.05) // if no imput and the swerve drive is still going:
            stopModules(); // stop the swerve
        

        leftStick = leftStick.rotateBy(TimesNegativeOne.ForwardOffset);
        leftStick = TimesNegativeOne.invert(leftStick, TimesNegativeOne.XAxis, TimesNegativeOne.YAxis);

        rotTarget = heading.getDegrees();

        var ctrl = new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(leftStick.getX() * speedAdjust)
            .withVelocityY(leftStick.getY() * speedAdjust)
            .withTargetDirection(heading);
        ctrl.HeadingController.setPID(
            SwerveDriveConstants.PIDConstants.AIM_kP.get(),
            SwerveDriveConstants.PIDConstants.AIM_kI.get(),
            SwerveDriveConstants.PIDConstants.AIM_kD.get()
            // SwerveDriveConstants.PIDConstants.AIM_GAINS.kP,
            // SwerveDriveConstants.PIDConstants.AIM_GAINS.kI,
            // SwerveDriveConstants.PIDConstants.AIM_GAINS.kD
        );
        io.setControl(ctrl);
        // SmartDashboard.putBoolean("drift correction", true);
    }

    public void driveFieldAngleSIP(Translation2d leftStick, Rotation2d heading) {
        
       rotTarget = heading.getDegrees();
        var ctrl = new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(leftStick.getX() * speedAdjust)
            .withVelocityY(leftStick.getY() * speedAdjust)
            .withTargetDirection(heading);
        ctrl.HeadingController.setPID(
            SwerveDriveConstants.PIDConstants.AIM_kP.get(),
            SwerveDriveConstants.PIDConstants.AIM_kI.get(),
            SwerveDriveConstants.PIDConstants.AIM_kD.get()
        );
        io.setControl(ctrl);
    }
    
    public void driveIntake(Translation2d leftStick, boolean invertRotation){
        // if (invert){
        //     Translation2d stick = new Translation2d(-leftStick.getX(), -leftStick.getY());
        //     driveFieldAngle(stick, heading);

        // } else{
        //     driveFieldAngle(leftStick, heading);
        // }
        double speed = leftStick.getNorm();

        if(speed < 0.3) {
            driveWithInput(leftStick, new Translation2d(), true);
        } else {



            Rotation2d heading = new Rotation2d(leftStick.getX(), -leftStick.getY());//.r otateBy(Rotation2d.fromDegrees(90));

            heading = heading.rotateBy(Rotation2d.fromDegrees(270));

            driveFieldAngle(leftStick, heading);
        }
    }

    public void driveIntakeOrientation(Translation2d leftStick, Translation2d rightStick){
        // if (invert){
        //     Translation2d stick = new Translation2d(-leftStick.getX(), -leftStick.getY());
        //     driveFieldAngle(stick, heading);

        // } else{
        //     driveFieldAngle(leftStick, heading);
        // }
        double speed = rightStick.getNorm();

        if(speed < 0.3) {
            driveWithInput(leftStick, new Translation2d(), true);
        } else {



            Rotation2d heading = new Rotation2d(rightStick.getX(), rightStick.getY());//.r otateBy(Rotation2d.fromDegrees(90));

            if(TimesNegativeOne.isRed) {
                heading = heading.rotateBy(Rotation2d.fromDegrees(-90));
            } else {
                heading = heading.rotateBy(Rotation2d.fromDegrees(90));
            }
            
            rotTarget = heading.getDegrees();

            driveFieldAngle(leftStick, heading);
        }
    }


    // Drive with the robot facing towards a specific position
    public void driveFacingPosition(Translation2d leftStick, Translation2d fieldPos, double aimLeadTime) {
        Translation2d robotSpeedYOnly = new Translation2d(0, chassisSpeeds.vyMetersPerSecond);
        double yDistance = Math.abs(getPose2d().getTranslation().getY() - fieldPos.getY());
        if ((chassisSpeeds.vyMetersPerSecond >0 &&getPose2d().getTranslation().getY() >4)||(chassisSpeeds.vyMetersPerSecond <0 &&getPose2d().getTranslation().getY() <4)){
        if (Math.abs(chassisSpeeds.vyMetersPerSecond) > 0.2) {
            if (TimesNegativeOne.isRed){
                    robotSpeedYOnly = new Translation2d(-SwerveDriveConstants.FAR_OFFSET.get() * yDistance * (getPose2d().getTranslation().getX()-7.28989525), chassisSpeeds.vyMetersPerSecond);
            } else {
                robotSpeedYOnly = new Translation2d((getPose2d().getTranslation().getX())* yDistance* SwerveDriveConstants.FAR_OFFSET.get(), chassisSpeeds.vyMetersPerSecond);
            }
        } }
        Translation2d fieldPosLead = robotSpeedYOnly.times(aimLeadTime).plus(fieldPos);
        Rotation2d ang = getPose2d().getTranslation().minus(fieldPosLead).getAngle();
        Pose2d fieldPosLeadLog = new Pose2d(fieldPosLead, new Rotation2d());
        Logger.recordOutput("Lead Aim", fieldPosLeadLog);
        driveFieldAngle(leftStick, ang);
    }

    public void offsetOdoPosition(Transform2d offset) {
        // Manually performing an addittion on the pose
        // WHY doesn't WPILIB have the ability to not transform poses
        Pose2d new_pose = new Pose2d(
            new Translation2d(
                state.currentPose.getX() + offset.getX(),
                state.currentPose.getY() + offset.getY()
            ),
            state.currentPose.getRotation()
        );
        this.io.resetPose(new_pose);
    }

    public void defenseXPosition(){
        io.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    public void stopDefenseXPosition(){
        stopModules();
    }

    public void driveFacingPosition(Translation2d leftStick, Translation2d fieldPos) {
        // Calculate the angle between the current position and the lead position
        //Rotation2d ang = getPose2d().getTranslation().minus(fieldPos).getAngle();
        Rotation2d ang = new Rotation2d(0,1);
        System.out.println(ang);

        driveFieldAngle(leftStick, ang);
    }

    public Pose2d getCurrentPose(){
        return state.currentPose;
    }

    public void driveRelativeLockedAngle(Translation2d leftStick, Rotation2d heading) {
        leftStick = leftStick.rotateBy(heading);

        var ctrl = new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(leftStick.getX() * speedAdjust)
            .withVelocityY(leftStick.getY() * speedAdjust)
            .withTargetDirection(heading);
        // ctrl.HeadingController.setPID(
        //     DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kP,
        //     DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kI,
        //     DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kD
        // );
        io.setControl(ctrl);
    }

    public void activateLuigiMode() {
        io.setLimits(20);
    }

    public void deactivateLuigiMode() {
        io.setLimits(SwerveDriveConstants.Configurations.SLIP_CURRENT);
    }

    public boolean rotateToTarget(double angle) {
        io.setControl(new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(angle)));

        if (Math.abs(angle - getGyroAngle()) < 5.0) {
            return true;
        }

        return false;
    }

    public boolean isStopped() {
        return lastOdomSpeed < AutoConstants.STOP_VELOCITY;
    }

    public void driveWithInputRotation(Translation2d leftStick, Rotation2d rot) {
    
        leftStick = leftStick.rotateBy(TimesNegativeOne.ForwardOffset);

        io.setControl(new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(leftStick.getX() * -speedAdjust)
                .withVelocityY(leftStick.getY() * speedAdjust)
                .withTargetDirection(rot));
    }

    public double getGyroAngle() {
        return getPose2d().getRotation().getRadians();
    }

    public Pose2d getPose2d() {
        if(state.currentPose == null)
            return initalPose2d;
        return state.currentPose;
    }

    public Supplier<Pose2d> getPoseSupplier() {
        return () -> this.getPose2d();
    }

    public void resetGyro() {
        io.tareEverything();
        robotKnowsWhereItIs = false;
        rotTarget = 0;
        // vision.resetRotations();
    }


    public void softStop() {
        stopped = true;
        io.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        ); // stop the modules without breaking
    }

    public void stopModules() {
        // stopped = true;
        // swerveDriveTrain.setControl(new SwerveRequest.SwerveDriveBrake());
        softStop();
    }

    public void enableRotationOverride(Translation2d fieldTarget, double aimLeadTime, Translation2d fieldPos) {
        Translation2d robotSpeedYOnly = new Translation2d(0, chassisSpeeds.vyMetersPerSecond);

        Translation2d fieldPosLead = robotSpeedYOnly.times(aimLeadTime).plus(fieldPos);
        m_rotationOverrideTarget = fieldPosLead;
        m_useRotationOverride = true;
    }

    public void disableRotationOverride() {
        m_useRotationOverride = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run\
        SmartDashboard.putNumber("Gyro", (getGyroAngle() * 180) / Math.PI);
        SmartDashboard.putNumber("RotTartget", rotTarget);

        io.updateInputs(state);
        Logger.processInputs("SwerveDrive", state);
        
        vision.setLastOdomPose(state.currentPose);
        setLastOdomSpeed(state.currentPose, state.lastPose, state.odometryRate);

        if (state.speeds != null) {
            this.chassisSpeeds = state.speeds;
        } else {
            this.chassisSpeeds = new ChassisSpeeds();
        }
        
        if (vision.isTag()) {
            Pose2d pose = vision.getPose2d();
            if (!robotKnowsWhereItIs) {
                robotKnowsWhereItIs = true;
                Pose2d curPose = getPose2d();
                rotTarget += pose.getRotation().getDegrees() - curPose.getRotation().getDegrees();
            }

            io.addVisionMeasurement(vision.getPosesToAdd());
            io.updateInputs(state);
            Logger.processInputs("SwerveDrive", state);

            vision.setLastOdomPose(state.currentPose);
            setLastOdomSpeed(state.currentPose, state.lastPose, state.odometryRate);
        }

        // if(e.isPresent())
    }

    private void reset_index() {
        gear_index = SwerveDriveConstants.STARTING_GEAR; // however we wish to initialize the gear (What gear does the
                                                         // robot start in?)
    }

    public void shiftDown() {
        if (gear_index == -1 || gear_index >= SwerveDriveConstants.GEARS.length)
            reset_index(); // If outof bounds: reset index
        int i = gear_index - 1;
        if (i == -1)
            i = 0;
        setPercentOutput(SwerveDriveConstants.GEARS[i]);
        gear_index = i;
    }

    public void shiftUp() {
        if (gear_index == -1 || gear_index >= SwerveDriveConstants.GEARS.length)
            reset_index(); // If outof bounds: reset index
        int i = gear_index + 1;
        if (i == SwerveDriveConstants.GEARS.length)
            i = SwerveDriveConstants.GEARS.length - 1;
        setPercentOutput(SwerveDriveConstants.GEARS[i]);
        gear_index = i;
    }


    public void setPercentOutput(double speed) {
        speedAdjust = SwerveDriveConstants.MAX_SPEED_MEETERS_PER_SEC * speed;
        gear_index = -1;
    }

    public void setToSlow() {
        setPercentOutput(SwerveDriveConstants.SLOW_SPEED);
        gear_index = 0;
    }

    public void setToFast() {
        setPercentOutput(SwerveDriveConstants.FAST_SPEED);
        gear_index = 1;
    }

    public void setToTurbo() {
        setPercentOutput(SwerveDriveConstants.TURBO_SPEED);
        gear_index = 2;
    }

    public void shiftUpRot() {
        rotSpeedAdjust = SwerveDriveConstants.ROTATION_SPEED;
    }

    public void shiftDownRot() {
        rotSpeedAdjust = SwerveDriveConstants.MIN_ROT_SPEED;
    }

    private int tmp_gear_index = SwerveDriveConstants.STARTING_GEAR;

    public void startSlowPeriod() {
        tmp_gear_index = gear_index;
        setToSlow();
    }

    public void startTurboPeriod() {
        tmp_gear_index = gear_index;
        setToTurbo();
    }

    public void endSlowPeriod() {
        setPercentOutput(SwerveDriveConstants.GEARS[tmp_gear_index]);
        gear_index = tmp_gear_index;
    }



    public void setLastOdomSpeed(Pose2d curPose, Pose2d lastPose, double freq){
        if(curPose != null && lastPose != null){
            lastOdomSpeed = curPose.getTranslation().getDistance(lastPose.getTranslation())/freq;
        }
    }

    @AutoLogOutput(key="SwerveDrive/speed ")
    public double getOdometrySpeed() {
        return lastOdomSpeed;
    }
    


    @Override
    public String getName() {
        return "Swerve Drive Controller";
    }

    @Override
    public Status diagnosticStatus() {
        Status status = new Status();

        // status.addReport(ReportLevel.INFO,
        //         "Don't know how to diganose new CTRE swerve systems. please check under the CAN(t) section for more detailed information about the swerves there.");

        return status;
    }
}