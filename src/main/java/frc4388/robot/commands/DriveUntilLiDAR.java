package frc4388.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.Lidar;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.TimesNegativeOne;

// Command to repeat a joystick movement for a specific time.
public class DriveUntilLiDAR extends Command {
    private final SwerveDrive swerveDrive;
    private final Translation2d leftStick;
    private final Translation2d rightStick;
    private final Lidar m_lidar;
    private final double mindistance;
    private final boolean robotRelative;

    public DriveUntilLiDAR(
        SwerveDrive swerveDrive, 
        Translation2d leftStick, 
        Translation2d rightStick, 
        Lidar lidar,
        double mindistance,
        boolean robotRelative) {
            addRequirements(swerveDrive);

        this.swerveDrive = swerveDrive;
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.m_lidar = lidar;
        this.mindistance = mindistance;
        this.robotRelative = robotRelative;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveDrive.driveFine(leftStick, rightStick, 0.3);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_lidar.getDistance()) < mindistance) {
            swerveDrive.softStop();
            return true;
        }
        return false;
    }
}