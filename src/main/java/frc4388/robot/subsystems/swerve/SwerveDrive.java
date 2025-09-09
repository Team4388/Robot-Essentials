// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants.AutoConstants;
import frc4388.robot.subsystems.vision.Vision;
import frc4388.utility.compute.TimesNegativeOne;
import frc4388.utility.status.Status;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

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
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
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
        io.resetPose(pose);
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
        // if (leftStick.getNorm() < 0.05 && stopped == false) // if no imput and the
        // swerve drive is still going:
        // stopModules(); // stop the swerve

        // if (leftStick.getNorm() < 0.05) //if no imput
        // return; // don't bother doing swerve drive math and return early.

        leftStick = leftStick.rotateBy(TimesNegativeOne.ForwardOffset);

        io.setControl(new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(leftStick.getX() * -speedAdjust)
                .withVelocityY(leftStick.getY() * speedAdjust)
                .withTargetDirection(rot));
        // double
    }

    public double getGyroAngle() {
        return getPose2d().getRotation().getRadians();
    }

    public Pose2d getPose2d() {
        if(state.currentPose == null)
            return initalPose2d;
        return state.currentPose;
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run\
        SmartDashboard.putNumber("Gyro", (getGyroAngle() * 180) / Math.PI);
        SmartDashboard.putNumber("RotTartget", rotTarget);

        io.updateInputs(state);
        Logger.processInputs("SwerveDrive", state);
        
        vision.setLastOdomPose(state.currentPose);
        setLastOdomSpeed(state.currentPose, state.lastPose, state.odometryRate);

        if (vision.isTag()) {
            Pose2d pose = vision.getPose2d();
            if (!robotKnowsWhereItIs) {
                robotKnowsWhereItIs = true;
                Pose2d curPose = getPose2d();
                rotTarget += pose.getRotation().getDegrees() - curPose.getRotation().getDegrees();
            }

            io.addVisionMeasurement(vision.getPosesToAdd());
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

