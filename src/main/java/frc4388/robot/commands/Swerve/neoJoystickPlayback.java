package frc4388.robot.commands.Swerve;

import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.DataUtils;
import frc4388.utility.UtilityStructs.AutoRecordingControllerFrame;
import frc4388.utility.UtilityStructs.AutoRecordingFrame;
import frc4388.utility.controller.VirtualController;


/**
 * The NEO autonomus playback system, designed based the old {@link JoystickPlayback} System but with {@link VirtualController}s 
 * @author Zachary Wilke
 */
public class neoJoystickPlayback extends Command {
    private final SwerveDrive                   swerve;
    private final VirtualController[]           controllers;
    private final ArrayList<AutoRecordingFrame> frames         = new ArrayList<>();
    private final Supplier<String>              filenameGetter;
    private       String                        filename;
    private       int                           frame_index    = 0;
    private       long                          startTime      = 0;
    private       long                          playbackTime   = 0;
    private       boolean                       m_finished     = false; // ! There is no better way.
    private       boolean                       m_shouldfree   = false; // should free memory on ending
    
    private byte  m_numAxes = 0;
    private byte  m_numPOVs = 0;
    private byte  m_numControllers = 0;
    private short m_numFrames = -1;

    /**
     * Creates an new NEO Joystick Playback with specifyed pramiters.
     * @param swerve m_robotSwerveDrive
     * @param filenameGetter a String Supplier, designed for quickly changing auto names in shuffle board. 
     * @param controllers an <b>Order-Specific</b> Array of Virtual controllers, index 0 means driver, index 1 means operator, etc.
     * @param shouldfree Unloads the auto on compleation or intruption.
     * @param instantload Load the auto on object instantiation
     */
    public neoJoystickPlayback(SwerveDrive swerve, Supplier<String> filenameGetter, VirtualController[] controllers, boolean shouldfree, boolean instantload) {
        this.swerve = swerve;
        this.filenameGetter = filenameGetter;
        this.controllers = controllers;
        this.m_shouldfree = shouldfree;

        if (instantload) loadAuto();
        addRequirements(this.swerve);
    }

    /**
     * Creates an new NEO Joystick Playback with specifyed pramiters.
     * @param swerve m_robotSwerveDrive
     * @param filename a String containing the name of the auto file you wish to playback. 
     * @param controllers an <b>Order-Specific</b> Array of Virtual controllers, index 0 means driver, index 1 means operator, etc.
     * @param shouldfree unloads the auto on compleation or intruption.
     * @param instantload load the auto on object instantiation
     */
    public neoJoystickPlayback(SwerveDrive swerve, String filename, VirtualController[] controllers, boolean shouldfree, boolean instantload) {
        this(swerve, () -> filename, controllers, shouldfree, instantload);
    }

    /**
     * Creates an new NEO Joystick Playback with specifyed pramiters.
     * @param swerve m_robotSwerveDrive
     * @param filenameGetter a String Supplier, designed for quickly changing auto names in shuffle board.
     * @param controllers an <b>Order-Specific</b> Array of Virtual controllers, index 0 means driver, index 1 means operator, etc.
     */
    public neoJoystickPlayback(SwerveDrive swerve, Supplier<String> filenameGetter, VirtualController[] controllers) {
        this(swerve, filenameGetter, controllers, true, false);
    }

    /**
     * Creates an new NEO Joystick Playback with specifyed pramiters.
     * @param swerve m_robotSwerveDrive
     * @param filename a String containing the name of the auto file you wish to playback. 
     * @param controllers an <b>Order-Specific</b> Array of Virtual controllers, index 0 means driver, index 1 means operator, etc.
     */
    public neoJoystickPlayback(SwerveDrive swerve, String filename, VirtualController[] controllers) {
        this(swerve, () -> filename, controllers, true, false);
    }
    
    /**
     * Load the auto file from disk into memory
     * @return Returns true if loading was successful, else wise; return false
     * @implNote if the auto is already loaded, it will return true. 
     */
    public boolean loadAuto() {
        filename = filenameGetter.get();
        try (FileInputStream stream = new FileInputStream("/home/lvuser/autos/" + filename)) {
            if (m_numFrames != -1 && m_numFrames == frames.size()) {
                System.out.println("AUTOPLAYBACK: Auto Already loaded.");
                return true;
            }
            
            m_numAxes = stream.readNBytes(1)[0];
            m_numPOVs = stream.readNBytes(1)[0];
            m_numControllers = stream.readNBytes(1)[0];
            m_numFrames = DataUtils.byteArrayToShort(stream.readNBytes(2));

            if (m_numControllers > controllers.length) {
                System.out.println("AUTOPLAYBACK: The auto file `" + filename + "` wants " + m_numControllers 
                + " virtual controllers but only " + controllers.length + " were given");
                return false;
            }

            for (int i = 0; i < m_numFrames; i++) {
                AutoRecordingFrame frame = new AutoRecordingFrame();
                for (int j = 0; j < m_numControllers; j++) {
                    AutoRecordingControllerFrame controllerFrame = new AutoRecordingControllerFrame();
                    double[] axes = new double[m_numAxes];
                    for (int k = 0; k < m_numAxes; k++) { // we love third level for loops.
                        axes[k] = DataUtils.byteArrayToDouble(stream.readNBytes(8));
                    }
                    short button = DataUtils.byteArrayToShort(stream.readNBytes(2));
                    short[] POV = new short[m_numPOVs];
                    for (int k = 0; k < m_numPOVs; k++) {
                        POV[k] = DataUtils.byteArrayToShort(stream.readNBytes(2));
                    }
                    controllerFrame.axes = axes;
                    controllerFrame.button = button;
                    controllerFrame.POV = POV;
                    frame.controllerFrames[j] = controllerFrame;
                }
                frame.timeStamp = DataUtils.byteArrayToInt(stream.readNBytes(4));
                frames.add(frame);
            }

            System.out.println("AUTOPLAYBACK: Read Auto `" + filename + "` that is " + m_numFrames + " frames long");
            return true;

        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("AUTOPLAYBACK: Unable to read auto file `" + filename + '`');
            return false;
        }
    }
    
    /**
     * Unloads the auto.
     */
    public void unloadAuto() {
        System.out.println("AUTOPLAYBACK: Auto unloaded");
        frames.clear();
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        playbackTime = 0;
        frame_index = 0;

        m_finished = !loadAuto();
    }

    @Override
    public void execute() {
        if (frame_index >= m_numFrames) m_finished = true;
        if (m_finished) return;

        // if (frame_index == 0) {
        //    startTime = System.currentTimeMillis();
        //    playbackTime = 0;
        // } else {
        //    playbackTime = System.currentTimeMillis() - startTime;
        // }

        AutoRecordingFrame frame = frames.get(frame_index);
        for (int i = 0; i < controllers.length; i++) {
            AutoRecordingControllerFrame controllerFrame = frame.controllerFrames[i];
            controllers[i].setFrame(controllerFrame.axes, controllerFrame.button, controllerFrame.POV);
            if (i == 0) {
                this.swerve.driveWithInput(
                    new Translation2d(controllers[i].getRawAxis(0), controllers[i].getRawAxis(1)),
                    new Translation2d(controllers[i].getRawAxis(4), controllers[i].getRawAxis(5)),
                    true);
            }
        }
        frame_index++;
    }

    @Override
    public void end(boolean interrupted) {
        for (VirtualController controller : controllers) controller.zeroControls();
        swerve.stopModules();
        if (m_shouldfree) unloadAuto();
    }
    
    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
