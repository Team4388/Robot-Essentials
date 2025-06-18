package frc4388.robot.commands;

import java.time.Instant;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.TimesNegativeOne;

// Command to repeat a joystick movement for a specific time.
public class MoveUntilSuply extends Command {
    private final SwerveDrive swerveDrive;
    private final Translation2d leftStick;
    private final Translation2d rightStick;
    private final Supplier<Boolean> truth;
    private final boolean robotRelative;

    public MoveUntilSuply(
        SwerveDrive swerveDrive, 
        Translation2d leftStick, 
        Translation2d rightStick, 
        Supplier<Boolean> truth,
        boolean robotRelative) {
            addRequirements(swerveDrive);

        this.swerveDrive = swerveDrive;
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.truth = truth;
        this.robotRelative = robotRelative;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveDrive.driveWithInput(leftStick, rightStick, !robotRelative);
    }

    @Override
    public boolean isFinished() {
        return truth.get();
    }
}