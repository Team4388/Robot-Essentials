// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Swerve;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.commands.PID;
import frc4388.robot.subsystems.swerve.SwerveDrive;

public class StayInPosition extends PID {
    SwerveDrive drive;
    Translation2d driveTranslation = new Translation2d();

    public StayInPosition(SwerveDrive drive) {
        super(0.3, 0.0, 0.0, 0.0, 1);
        this.drive = drive;
        addRequirements(drive);
    }

    public void goToTargetPose(Pose2d targetPose) {
        Pose2d currentPose = drive.getCurrentPose();
        double translationX = targetPose.getX() - currentPose.getX();
        double translationY = targetPose.getY() - currentPose.getY();
        if (translationX > 0.2){
        //If below is changed make this change too so it never goes over 1
          translationX = 0.2;
        }
        if (translationY > 0.2){
        //Same here
          translationY = 0.2;
        }
        if (Math.abs(translationX) < 0.08 && Math.abs(translationY) < 0.08) {
            driveTranslation = new Translation2d();
        } else {
            //TODO: Tweak for best corrector against another bot
            driveTranslation = new Translation2d(translationX * 4.5, translationY * 4.5); 
        }

        drive.driveFieldAngleSIP(driveTranslation, targetPose.getRotation());
    }

    @Override
    public double getError() {
        return 0;
    }

    @Override
    public void runWithOutput(double output) {}
}