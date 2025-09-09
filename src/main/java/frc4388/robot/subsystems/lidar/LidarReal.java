package frc4388.robot.subsystems.lidar;

import edu.wpi.first.wpilibj.Counter;
import frc4388.robot.constants.Constants.LiDARConstants;

// https://girlsofsteeldocs.readthedocs.io/en/latest/technical-resources/sensors/LIDAR-Lite-Distance-Sensor.html#minimal-roborio-interface
public class LidarReal implements LidarIO {

    
    private Counter LidarPWM;

    public LidarReal(int port) {
        LidarPWM = new Counter(port);
        LidarPWM.setMaxPeriod(1.00); //set the max period that can be measured
        LidarPWM.setSemiPeriodMode(true); //Set the counter to period measurement
        LidarPWM.reset();
    }

    @Override
    public void updateInputs(LidarState state) {
        
        if(LidarPWM.get() < 1)
            state.distance = -1;
        else
            state.distance = (LidarPWM.getPeriod() * LiDARConstants.SECONDS_TO_MICROS) / LiDARConstants.LIDAR_MICROS_TO_CM;
    }
}
