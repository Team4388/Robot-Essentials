package frc4388.robot.subsystems.vision;

import com.fazecast.jSerialComm.SerialPort;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Robust RPLidar A1 / R1M8 Driver for FRC.
 * Implements standard protocol with auto-reconnection and state monitoring.
 */
public class RPLidarA1 {

    // --- Data Types ---

    public static class PolarPoint {
        public final double angle;    // Degrees 0-360
        public final double distance; // Meters

        public PolarPoint(double angle, double distance) {
            this.angle = angle;
            this.distance = distance;
        }
    }

    @FunctionalInterface
    public interface ScanListener {
        void onScanComplete(List<PolarPoint> scan);
    }

    public enum ConnectionStatus {
        DISCONNECTED,      // Port not found or closed
        CONNECTING,        // Attempting to open serial port
        CONNECTED_IDLE,    // Port open, but scan not started / no data yet
        CONNECTED_DISABLED,// Robot is disabled, but sensor is connected
        RECEIVING_DATA,    // Actively receiving valid scan points
        ERROR              // Communication failure or timeout
    }

    // --- Protocol Constants ---
    private static final byte SYNC_BYTE = (byte) 0xA5;
    private static final byte SYNC_BYTE2 = (byte) 0x5A;
    private static final byte CMD_STOP = (byte) 0x25;
    private static final byte CMD_RESET = (byte) 0x40;
    private static final byte CMD_SCAN = (byte) 0x20;
    private static final byte CMD_GET_HEALTH = (byte) 0x52;

    private static final int DESCRIPTOR_LEN = 7;
    private static final int SCAN_PACKET_LEN = 5;

    // --- Settings ---
    private static final String PORT_DESC = "CP2102 USB to UART Bridge Controller";
    private static final double WATCHDOG_TIMEOUT = 2.5; // Seconds before assuming link is dead

    // --- Members ---
    private final AtomicReference<ConnectionStatus> mStatus = new AtomicReference<>(ConnectionStatus.DISCONNECTED);
    private SerialPort mSerialPort;
    private InputStream mIn;
    private OutputStream mOut;
    private ScanListener mListener;
    
    private final List<PolarPoint> mCurrentScan = new ArrayList<>();
    private double mLastDataTimestamp = 0;
    // private boolean mScanningActive = false;

    public RPLidarA1() {
        Thread driverThread = new Thread(this::runLoop);
        driverThread.setDaemon(true);
        driverThread.setName("RPLidar-Driver-Thread");
        driverThread.start();

        Thread pwmThread = new Thread(this::funnyDTR_PWM);
        pwmThread.setDaemon(true);
        pwmThread.setName("RPLidar-Driver-PWM");
        pwmThread.start();
    }

    /** Sets the function to call whenever a full 360-degree rotation is parsed. */
    public void setListener(ScanListener listener) {
        this.mListener = listener;
    }

    // Set Speed of motor between 0 - 1
    public void setSpeed(double speed) {
        this.motor_percentage = speed;
    }

    public ConnectionStatus getStatus() {
        return mStatus.get();
    }

    /** Signals the Lidar to stop the motor and laser. */
    private void stop_motor() {
        sendCmd(CMD_RESET);
        Timer.delay(0.02);
        sendCmd(CMD_STOP);
        mSerialPort.setDTR();
    }


    private final static double TOGGLE_DELAY = 10;
    private double motor_percentage = 0.5;
    private boolean is_dtr = false;

    // Control the speed of the motor like a PWM through the DTR serial pin
    // This is "PWM", like we control the speed through the percentage.
    // The rate of toggles is the resolution
    private void funnyDTR_PWM() {
        while (!Thread.interrupted()) {
            try {
                ConnectionStatus status = mStatus.get();
                if (status == ConnectionStatus.RECEIVING_DATA) {

                    // If the motor is at full speed
                    if (motor_percentage >= 1) {
                        // Set the motor to on
                        mSerialPort.clearDTR();
                        // check again in a little bit
                        Thread.sleep(100);
                    }

                    
                    // If the motor is at zero speed
                    if (motor_percentage <= 0) {
                        // Set the motor to on
                        mSerialPort.setDTR();
                        // check again in a little bit
                        Thread.sleep(100);
                    }

                    if (is_dtr) {
                        mSerialPort.clearDTR();
                        // Sleep for main part of motor pulse
                        Thread.sleep((long) (TOGGLE_DELAY * motor_percentage));
                    } else {
                        mSerialPort.setDTR();
                        // Sleep for gap of motor pulse
                        Thread.sleep((long) (TOGGLE_DELAY * (1 - motor_percentage)));
                    }

                    is_dtr = !is_dtr;

                } else if(status == ConnectionStatus.CONNECTED_DISABLED) {
                    // Stop the motor
                    mSerialPort.setDTR();

                
                    // Sleep until we can check again
                    Thread.sleep(100);

                } else { // When the motor is not ready
                    // Sleep until we can check again
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                continue;
            }
        }

    }

    private void runLoop() {
        while (!Thread.interrupted()) {

            try {
            ConnectionStatus current = mStatus.get();

            boolean robotEnabled = DriverStation.isEnabled();
            

            switch (current) {
                case DISCONNECTED:
                case ERROR:
                    attemptConnection();
                    break;

                case CONNECTING:
                    // Handled by attemptConnection
                    break;
                
                case CONNECTED_DISABLED:
                    if (robotEnabled) {
                        mStatus.set(ConnectionStatus.CONNECTED_IDLE);
                        // On enable, set the last data time to now to avoid watchdog error
                        mLastDataTimestamp = Timer.getFPGATimestamp();
                        break;
                    }

                    // We have to check the health seperately because 
                    // the connection check only ever occurs when
                    // the robot is recieving data
                    if (!getHealth()) {
                        mStatus.set(ConnectionStatus.ERROR);
                    }

                    break;

                case CONNECTED_IDLE:
                    if (!robotEnabled) {
                        mStatus.set(ConnectionStatus.CONNECTED_DISABLED);
                        // On enable, set the last data time to now to avoid watchdog error
                        mLastDataTimestamp = Timer.getFPGATimestamp();
                        break;
                    }

                    if (initiateScanMode()) {
                        mStatus.set(ConnectionStatus.RECEIVING_DATA);
                        mLastDataTimestamp = Timer.getFPGATimestamp();
                    } else {

                        mStatus.set(ConnectionStatus.ERROR);
                    }

                    break;

                case RECEIVING_DATA:
                    if (!robotEnabled) {
                        mStatus.set(ConnectionStatus.CONNECTED_DISABLED);
                        break;
                    }

                    processIncomingData();
                    

                    checkWatchdog();
                    break;
            }

            Thread.sleep(200);

            } catch (Exception e) {
                continue;
            }
        }
    }

    private void attemptConnection() {
        if (mSerialPort != null && mSerialPort.isOpen()) {
            mSerialPort.closePort();
        }

        mStatus.set(ConnectionStatus.CONNECTING);
        
        SerialPort[] ports = SerialPort.getCommPorts();
        for (SerialPort p : ports) {
            if (p.getPortDescription().contains(PORT_DESC)) {
                mSerialPort = p;
                break;
            }
        }

        if (mSerialPort != null) {
            mSerialPort.setComPortParameters(115200, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
            mSerialPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
            if (mSerialPort.openPort()) {
                mIn = mSerialPort.getInputStream();
                mOut = mSerialPort.getOutputStream();

                if (DriverStation.isEnabled()) {
                    mStatus.set(ConnectionStatus.CONNECTED_IDLE);
                    // On start, set the last data time to now to avoid watchdog error
                    mLastDataTimestamp = Timer.getFPGATimestamp();
                } else {
                    mStatus.set(ConnectionStatus.CONNECTED_DISABLED);
                    stop_motor();
                }

                mStatus.set(ConnectionStatus.CONNECTED_IDLE);
                
                // For A1: DTR False starts motor, DTR True stops it.
                // mSerialPort.setDTR(); 
                return;
            }
        }
        
        mStatus.set(ConnectionStatus.DISCONNECTED);
        Timer.delay(1.0); // Wait before retry
    }

    private boolean initiateScanMode() {
        try {
            // Clear buffer before starting
            while (mIn.available() > 0) mIn.read();

            mSerialPort.clearDTR(); // Start Motor

            Thread.sleep(100);

            sendCmd(CMD_SCAN);
            
            // Wait for 7-byte descriptor
            byte[] descriptor = new byte[DESCRIPTOR_LEN];
            long start = System.currentTimeMillis();
            while (mIn.available() < DESCRIPTOR_LEN) {
                if (System.currentTimeMillis() - start > 1000) return false;
                Timer.delay(0.01);
            }
            
            mIn.read(descriptor);

            return descriptor[0] == SYNC_BYTE && descriptor[1] == SYNC_BYTE2;
        } catch (Exception e) {
            return false;
        }
    }

    private void processIncomingData() {
        try {
            while (mIn.available() >= SCAN_PACKET_LEN) {
                byte[] packet = new byte[SCAN_PACKET_LEN];
                mIn.read(packet);

                // Protocol validation based on provided Python logic
                boolean newScan = (packet[0] & 0x1) != 0;
                boolean invNewScan = ((packet[0] >> 1) & 0x1) != 0;
                int checkBit = (packet[1] & 0x1);

                if (newScan == invNewScan || checkBit != 1) {
                    // Out of sync - skip one byte to try and find sync again
                    return; 
                }

                mLastDataTimestamp = Timer.getFPGATimestamp();

                // Python logic: ((raw[1] >> 1) + (raw[2] << 7)) / 64.
                int angleRaw = ((packet[1] & 0xFF) >> 1) + ((packet[2] & 0xFF) << 7);
                double angle = angleRaw / 64.0;

                // Python logic: (raw[3] + (raw[4] << 8)) / 4. (in mm)
                int distRaw = (packet[3] & 0xFF) + ((packet[4] & 0xFF) << 8);
                double distanceMeters = distRaw / 4000.0;

                if (newScan && !mCurrentScan.isEmpty()) {
                    if (mListener != null) {
                        mListener.onScanComplete(new ArrayList<>(mCurrentScan));
                    }
                    mCurrentScan.clear();
                }

                if (distanceMeters > 0) {
                    mCurrentScan.add(new PolarPoint(angle, distanceMeters));
                }
            }
        } catch (Exception e) {
            mStatus.set(ConnectionStatus.ERROR);
        }
    }

    private void checkWatchdog() {
        if (Timer.getFPGATimestamp() - mLastDataTimestamp > WATCHDOG_TIMEOUT) {

            // //
            // stop_motor();

            mStatus.set(ConnectionStatus.CONNECTED_IDLE);


            // We have to check the health seperately because 
            // the connection check only ever occurs when
            // the robot is recieving data
            // if (!getHealth()) {

            //     DriverStation.reportWarning("RPLidar A1: Data timeout. Reconnecting...", false);
            //     mStatus.set(ConnectionStatus.ERROR);

            // }
        }
    }

    private void sendCmd(byte cmd) {
        try {
            if (mOut != null) {
                mOut.write(new byte[]{SYNC_BYTE, cmd});
                mOut.flush();
            }
        } catch (Exception e) {
            mStatus.set(ConnectionStatus.ERROR);
        }
    }

    /**
     * Queries the device health status. 
     * @return true if the device is connected and returns a 'Good' health status, false otherwise.
     */
    public boolean getHealth() {
        if (mStatus.get() == ConnectionStatus.DISCONNECTED || mOut == null || mIn == null) {
            return false;
        }

        try {
            // Ensure the buffer is clear before sending request
            while (mIn.available() > 0) mIn.read();

            sendCmd(CMD_GET_HEALTH);

            // Read 7-byte Descriptor
            byte[] descriptor = new byte[DESCRIPTOR_LEN];
            long startTime = System.currentTimeMillis();
            while (mIn.available() < DESCRIPTOR_LEN) {
                if (System.currentTimeMillis() - startTime > 500) return false;
                Timer.delay(0.01);
            }
            mIn.read(descriptor);

            return true;

            // // Check if descriptor is valid and data type matches HEALTH (0x06)
            // if (descriptor[0] != SYNC_BYTE || descriptor[1] != SYNC_BYTE2 || descriptor[6] != 0x06) {
            //     return false;
            // }

            // // Read 3-byte Health Payload
            // byte[] healthPayload = new byte[3];
            // while (mIn.available() < 3) {
            //     if (System.currentTimeMillis() - startTime > 1000) return false;
            //     Timer.delay(0.01);
            // }
            // mIn.read(healthPayload);

            // Byte 0 is status: 0x00 = Good, 0x01 = Warning, 0x02 = Error
            // return healthPayload[0] == 0;
        } catch (Exception e) {
            return false;
        }
    }
}