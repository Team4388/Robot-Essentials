package frc4388.robot.commands.Autos;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4388.robot.commands.Swerve.JoystickPlayback;
import frc4388.robot.subsystems.SwerveDrive;

public class PlaybackChooser {
    private final ArrayList<SendableChooser<Command>> m_choosers    = new ArrayList<>();
    private       SendableChooser<Command>            m_playback    = null;
    private final ArrayList<ComplexWidget>            m_widgets     = new ArrayList<>();
    private final HashMap<String, Command>            m_commandPool = new HashMap<>();
    
    private final File        m_dir    = new File("/home/lvuser/autos/");
    private       int         m_cmdNum = 0;
    private final SwerveDrive m_swerve;

    // commands
    private Command m_noAuto = new InstantCommand();
    
    public PlaybackChooser(SwerveDrive swerve, Object... pool) {
        m_swerve = swerve;
    }

    public PlaybackChooser addOption(String name, Command option) {
        m_commandPool.put(name, option);
        return this;
    }

    public PlaybackChooser buildDisplay() {
        for (int i = 0; i < 10; i++) {
            appendCommand();
        }
        m_playback = m_choosers.get(0);
        nextChooser();

        // ! This does not work, why?
        Shuffleboard.getTab("Auto Chooser")
            .add("Add Sequence", new InstantCommand(() -> nextChooser()))
            .withPosition(4, 0);
        return this;
    }

    // This will be bound to a button for the time being
    public void appendCommand() {
        var chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("No Auto", m_noAuto);

        m_choosers.add(chooser);
        ComplexWidget widget = Shuffleboard.getTab("Auto Chooser")
            .add("Command: " + m_choosers.size(), chooser)
            .withSize(4, 1)
            .withPosition(0, m_choosers.size() - 1)
            .withWidget(BuiltInWidgets.kSplitButtonChooser);

        m_widgets.add(widget);
    }

    public void nextChooser() {
        SendableChooser<Command> chooser = m_choosers.get(m_cmdNum++);

        String[] dirs = m_dir.list();

        if(dirs != null){ // Fix funny error
            for (String auto : dirs) {
                chooser.addOption(auto, new JoystickPlayback(m_swerve, auto));
            }
        }

        
        for (var cmd_name : m_commandPool.keySet()) {
            chooser.addOption(cmd_name, m_commandPool.get(cmd_name));
        }
    }

    public Command getCommand() {
        Command command = m_playback.getSelected();
        command = command == null ? m_noAuto : command.asProxy();
        
        Command[] commands = new Command[m_cmdNum - 1];
        for (int i = 0; i < m_cmdNum - 1; i++) {
            Command command2 = m_choosers.get(i + 1).getSelected();
            command2 = command2 == null ? m_noAuto : command2.asProxy();

            commands[i] = command2.asProxy();
        }

        return command.andThen(commands);
    }
}
