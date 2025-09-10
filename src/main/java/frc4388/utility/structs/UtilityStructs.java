package frc4388.utility.structs;

public class UtilityStructs {
    public static class TimedOutput {
        public double leftX  = 0.0;
        public double leftY  = 0.0;
        public double rightX = 0.0;
        public double rightY = 0.0;

        public boolean OPLB;
        public boolean OPRB;

    
        public long timedOffset = 0;
    }
    public static class AutoRecordingControllerFrame {
        public double[] axes = new double[6];
        public short button = 0;
        public short[] POV = new short[1];

    }
    public static class AutoRecordingFrame {
        public AutoRecordingControllerFrame[] controllerFrames = new AutoRecordingControllerFrame[2];
        public int timeStamp;
    }
}
