package frc4388.utility;

import java.nio.ByteBuffer;

public class DataUtils {
    public static byte[] doubleToByteArray(double value) {
        byte[] bytes = new byte[8];
        ByteBuffer.wrap(bytes).putDouble(value);
        return bytes;
    }
    
    public static double byteArrayToDouble(byte[] bytes) {
        return ByteBuffer.wrap(bytes).getDouble();
    }

    public static byte[] intToByteArray(int value) {
        byte[] bytes = new byte[4];
        ByteBuffer.wrap(bytes).putInt(value);
        return bytes;
    }
    
    public static int byteArrayToInt(byte[] bytes) {
        return ByteBuffer.wrap(bytes).getInt();
    }

    public static byte[] shortToByteArray(short value) {
        byte[] bytes = new byte[2];
        ByteBuffer.wrap(bytes).putShort(value);
        return bytes;
    }
    
    public static short byteArrayToShort(byte[] bytes) {
        return ByteBuffer.wrap(bytes).getShort();
    }
}
