// package frc4388.robot.commands.Autos;

// import java.io.File;
// import java.util.ArrayList;
// import java.util.HashMap;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc4388.robot.commands.Swerve.JoystickPlayback;
// import frc4388.robot.commands.Swerve.neoJoystickPlayback;
// import frc4388.robot.subsystems.SwerveDrive;
// import frc4388.utility.controller.VirtualController;

// public class neoPlaybackChooser {
//     private final SendableChooser<String> m_teamChosser = new SendableChooser<String>();    
//     private final SendableChooser<String> m_possitionChosser = new SendableChooser<String>();
//     private final SendableChooser<String> m_autoNameChosser = new SendableChooser<String>();

//     private final SwerveDrive m_swerve;
//     private final VirtualController[] m_controllers;
//     // private final ArrayList<SendableChooser<Command>> m_choosers    = new ArrayList<>();
//     // private       SendableChooser<Command>            m_playback    = null;
//     private final ArrayList<ComplexWidget>            m_widgets     = new ArrayList<>();
//     // private final HashMap<String, Command>            m_commandPool = new HashMap<>();
    
//     // private final File        m_dir    = new File("/home/lvuser/autos/");
//     // private       int         m_cmdNum = 0;

//     // commands
//     private Command m_noAuto = new InstantCommand();
    
//     public neoPlaybackChooser(SwerveDrive swerve, VirtualController[] controllers) {
//         m_teamChosser.addOption("Red", "red");
//         m_teamChosser.setDefaultOption("Blue", "blue");
//         m_teamChosser.addOption("Nuetral", "nuetral");
//         m_possitionChosser.addOption("AMP", "amp");
//         m_possitionChosser.setDefaultOption("Center", "center");
//         m_possitionChosser.addOption("Source", "source");
//         m_swerve = swerve;
//         m_controllers = controllers;
//     }

//     public neoPlaybackChooser addOption(String name, String option) {
//         m_autoNameChosser.addOption(name, option);
//         return this;
//     }

//     // public PlaybackChooser buildDisplay() {
//     //     for (int i = 0; i < 10; i++) {
//     //         appendCommand();
//     //     }
//     //     m_playback = m_choosers.get(0);
//     //     nextChooser();

//     //     // ! This does not work, why?
//     //     Shuffleboard.getTab("Auto Chooser")
//     //         .add("Add Sequence", new InstantCommand(() -> nextChooser()))
//     //         .withPosition(4, 0);
//     //     return this;
//     // }

//     // This will be bound to a button for the time being
//     public void render() {
//         // var chooser = new SendableChooser<Command>();
//         // // chooser.setDefaultOption("No Auto", m_noAuto);

//         // m_choosers.add(chooser);
//         ComplexWidget widget = Shuffleboard.getTab("Neo Auto Chooser")
//             .add("Command: " + m_choosers.size(), chooser)
//             .withSize(4, 1)
//             .withPosition(0, m_choosers.size() - 1)
//             .withWidget(BuiltInWidgets.kSplitButtonChooser)
//             .withWidget(BuiltInWidgets.kComboBoxChooser);


//         m_widgets.add(widget);
//     }

//     // public void nextChooser() {
//     //     SendableChooser<Command> chooser = m_choosers.get(m_cmdNum++);

//     //     String[] dirs = m_dir.list();

//     //     if(dirs != null){ // Fix funny error
//     //         for (String auto : dirs) {
//     //             chooser.addOption(auto, new JoystickPlayback(m_swerve, auto));
//     //         }
//     //     }

        
//     //     for (var cmd_name : m_commandPool.keySet()) {
//     //         chooser.addOption(cmd_name, m_commandPool.get(cmd_name));
//     //     }
//     // }
    
//     public String autoName() {
//         return m_teamChosser.getSelected() + "_" + m_possitionChosser.getSelected() + "_" + m_autoNameChosser.getSelected() + ".auto";
//     }

//     public Command getCommand() {
//         return new neoJoystickPlayback(m_swerve, autoName(), m_controllers, true, true);
//     }
// }
