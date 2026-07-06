package frc4388.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    public ClimberIO io;
    ClimberStateAutoLogged state = new ClimberStateAutoLogged();

    public Climber(
        ClimberIO io
    ) {
        this.io = io;
    }

    ClimberMode mode = ClimberMode.Retracting;

    public enum ClimberMode {
        Extended,
        Retracting
    }

    public void deploy() {
        mode = ClimberMode.Extended;
    }

    public void retract() {
        mode = ClimberMode.Retracting;
    }

    public void toggleDeployed() {
        switch (mode) {
            case Extended:
                mode = ClimberMode.Retracting;
                break;
            case Retracting:
                mode = ClimberMode.Extended;
                break;
        }
    }    

    @Override
    public void periodic() {
        
        switch (mode) {
            case Extended:
                io.setClimberDistance(state, Inches.of(ClimberConstants.CLIMBER_RETRACT_SPEED.get()));
                break;
            case Retracting:
                io.setPercentOutput(state, ClimberConstants.CLIMBER_RETRACT_SPEED.get());
                break;
        }

        Logger.processInputs("Climber", state);

        io.updateInputs(state);

    }
}


