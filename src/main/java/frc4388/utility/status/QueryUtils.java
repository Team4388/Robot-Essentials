package frc4388.utility.status;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.filter.Debouncer;

public class QueryUtils {
    public static boolean isDebounceOk(@SuppressWarnings("rawtypes") StatusSignal status) {
        Debouncer connectedDebounce = new Debouncer(0.5);
        status.refresh();
        return connectedDebounce.calculate(status.getStatus().isOK());
    };
}
