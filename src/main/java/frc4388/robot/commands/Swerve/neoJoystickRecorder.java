package frc4388.robot.commands.Swerve;

import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.DataUtils;
import frc4388.utility.UtilityStructs.AutoRecordingControllerFrame;
import frc4388.utility.UtilityStructs.AutoRecordingFrame;
import frc4388.utility.controller.DeadbandedXboxController;

/**
 * The NEO autonomus recording system, designed based the old {@link JoystickRecorder} System but with {@link frc4388.utility.controller.VirtualController VirtualController}s 
 * @author Zachary Wilke
 */
public class neoJoystickRecorder extends Command {
    private final SwerveDrive swerve;
    private final XboxController[] controllers;
    private       String filename;
    private final Supplier<String> filenameGetter;
    private       long startTime = -1;
    private final ArrayList<AutoRecordingFrame> frames = new ArrayList<>();

    /**
     * Creates an new NEO Joystick Playback with specifyed pramiters.
     * @param swerve m_robotSwerveDrive
     * @param controllers an <b>Order-Specific</b> Array of Virtual controllers, index 0 means driver, index 1 means operator, etc.
     * @param filenameGetter a String Supplier, designed for quickly changing auto names in shuffle board.
     */
    public neoJoystickRecorder(SwerveDrive swerve, DeadbandedXboxController[] controllers, Supplier<String> filenameGetter) {
        this.swerve = swerve;
        this.controllers = controllers;
        this.filenameGetter = filenameGetter;
        this.filename = "";

        addRequirements(this.swerve);
    }

    /**
     * Creates an new NEO Joystick Playback with specifyed pramiters.
     * @param swerve m_robotSwerveDrive
     * @param controllers an <b>Order-Specific</b> Array of Virtual controllers, index 0 means driver, index 1 means operator, etc.
     * @param filename a String containing the name of the auto file you wish to playback. 
     */
    public neoJoystickRecorder(SwerveDrive swerve, DeadbandedXboxController[] controllers, String filename) {
        this(swerve, controllers, () -> filename);
    }

    @Override
    public void initialize() {
        frames.clear();

        this.startTime = System.currentTimeMillis();
        AutoRecordingFrame frame = new AutoRecordingFrame();
        frame.controllerFrames = new AutoRecordingControllerFrame[] {new AutoRecordingControllerFrame(), new AutoRecordingControllerFrame()};
        frames.add(frame);
        this.filename = this.filenameGetter.get();
    }


    @Override
    public void execute() {
        System.out.println("AUTORECORD: RECORDING");
        AutoRecordingFrame frame = new AutoRecordingFrame();
        frame.timeStamp = (int) (System.currentTimeMillis() - startTime);
        for (int i = 0; i < controllers.length; i++) {
            XboxController controller = controllers[i];
            AutoRecordingControllerFrame controllerFrame = new AutoRecordingControllerFrame();
            double[] axes = {controller.getLeftX(), controller.getLeftY(), 
                            controller.getLeftTriggerAxis(), controller.getRightTriggerAxis(),
                            controller.getRightX(), controller.getRightY()};
            short button = 0;
            for (int j = 0; j < 10; j++)
                if (controller.getRawButton(j+1))
                    button |= 1 << j;
            short[] POV = {(short)(controller.getPOV())};
            controllerFrame.axes = axes;
            controllerFrame.button = button;
            controllerFrame.POV = POV;
            frame.controllerFrames[i] = controllerFrame;
        }

        frames.add(frame);

        swerve.driveWithInput(new Translation2d(frame.controllerFrames[0].axes[0], frame.controllerFrames[0].axes[1]),
                            new Translation2d(frame.controllerFrames[0].axes[4], frame.controllerFrames[0].axes[5]), 
                            true); // Really jank way of doing this.
        
    }
    @Override
    public void end(boolean interrupted) {
        try (FileOutputStream stream = new FileOutputStream("/home/lvuser/autos/" + filename)) {
            // header: size of 0x5
            // byte Number of axes per controller
            // byte Number of POVs per controller
            // byte Number of controllers
            // short Number of frames 
            stream.write(new byte[]{6, 1, (byte) controllers.length}); 
            stream.write(DataUtils.shortToByteArray((short) frames.size())); 

            // frame
            // controller frame * number of controllers
            // int unix time stamp.
            for (AutoRecordingFrame frame : frames) {
                // controller frame
                // double axis * Number of axes per controller
                // short button states
                // short POV * Number of POVs per controller
                for (AutoRecordingControllerFrame controllerFrame: frame.controllerFrames) {
                    for (double axis: controllerFrame.axes) {
                        stream.write(DataUtils.doubleToByteArray(axis));
                    }
                    stream.write(DataUtils.shortToByteArray(controllerFrame.button));
                    for (short POV: controllerFrame.POV) {
                        stream.write(DataUtils.shortToByteArray(POV));
                    }
                }
                stream.write(DataUtils.intToByteArray(frame.timeStamp));
            }
        System.out.println("AUTORECORD: Wrote auto `" + filename + "` that is " + frames.size() + " frames long.");
        } catch (Exception e) {
            e.printStackTrace();
        } 
    }
}
