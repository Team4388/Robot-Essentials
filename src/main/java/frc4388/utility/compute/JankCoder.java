package frc4388.utility.compute;

import static edu.wpi.first.units.Units.Rotations;

import java.io.FileInputStream;
import java.io.FileOutputStream;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc4388.utility.configurable.ConfigurableDouble;

public class JankCoder {
    DutyCycleEncoder m_encoder;

    boolean loaded_rotations = false;
    int rotations = 0;
    double lastRotation = 0;
    final ConfigurableDouble offset;

    public JankCoder(int dio_channel, ConfigurableDouble offset) {
        this.offset = offset;
        m_encoder = new DutyCycleEncoder(dio_channel);
        loadRotations();
    }

    public void update() {
        // if(!m_encoder.isConnected()) {
        //     return;
        // }
            
        if(!loaded_rotations) {
            loadRotations();
        } else {
            double curRot = m_encoder.get();
            if(lastRotation - curRot > 0.5) {
                rotations += 1;
                saveRotations();
            } else if (curRot - lastRotation > 0.5) {
                rotations -= 1;
                saveRotations();
            }

            lastRotation = curRot;
        }
    }

    public double get() {
        return (double) rotations + m_encoder.get() + offset.get();
    }

    public Angle getRotations() {
        return Rotations.of(get());
    }
    
    public int getRotationCount() {
        return rotations;
    }
    
    public void resetRotations() {
        offset.set(-m_encoder.get());
        setRotations(0);
    }

    public void setRotations(int rotation) {
        rotations = rotation;
        saveRotations();
    }

    public boolean isConnected() {
        return m_encoder.isConnected();
    }

    public void saveRotations() {
        try (FileOutputStream stream = new FileOutputStream("/home/lvuser/encoder" + m_encoder.getSourceChannel())) {
            stream.write(DataUtils.intToByteArray(rotations));
        } catch (Exception e) {
            // e.printStackTrace();
            System.out.println("ENCODER: Unable to write to trim file `" + m_encoder.getSourceChannel() + "`!?!");
        }
    }

    public boolean loadRotations() {
        lastRotation = m_encoder.get();
        this.loaded_rotations = true;

        try (FileInputStream stream = new FileInputStream("/home/lvuser/encoder" + m_encoder.getSourceChannel())) {
            int fileValue = DataUtils.byteArrayToInt(stream.readNBytes(4));
            
            rotations = fileValue;
            // clampModify();
            // modified = false;

            // if (fileValue != currentValue) {
            //     // System.out.println("TRIMS: Loaded trim `" + trimName + "` has a value that is higher than or less than the bounds set for the trim, clamping...");
            //     // modified = true;
            // }
            return true;
        } catch (Exception e) {
            // e.printStackTrace();
            System.out.println("ENCODER: Unable to read encoder `" + m_encoder.getSourceChannel() + "`, using current value...");
            return false;
        }
        
    }
}
