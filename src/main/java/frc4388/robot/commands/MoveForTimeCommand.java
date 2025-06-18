package frc4388.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.TimesNegativeOne;

// Command to repeat a joystick movement for a specific time.
public class MoveForTimeCommand extends Command {
    private final SwerveDrive swerveDrive;
    private final Translation2d leftStick;
    private final Translation2d rightStick;
    private final long duration;
    private final boolean robotRelative;

    private Instant startTime;

    public MoveForTimeCommand(
        SwerveDrive swerveDrive, 
        Translation2d leftStick, 
        Translation2d rightStick, 
        long millis,
        boolean robotRelative) {
            addRequirements(swerveDrive);

        this.swerveDrive = swerveDrive;
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.duration = millis;
        this.robotRelative = robotRelative;
    }

    @Override
    public void initialize() {
        startTime = Instant.now();
    }

    @Override
    public void execute() {
        swerveDrive.driveWithInput(leftStick, rightStick, !robotRelative);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(startTime.toEpochMilli() - Instant.now().toEpochMilli()) > duration;
    }
}