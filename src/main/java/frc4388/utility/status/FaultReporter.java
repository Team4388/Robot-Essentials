package frc4388.utility.status;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import frc4388.robot.constants.Constants;
import frc4388.utility.status.Status.Report;
import frc4388.utility.status.Status.ReportLevel;

public class FaultReporter {

    private static final String REPORTS_HEADER = 
    "###############\n" + //
    "#.............#\n" + //
    "#...Reports...#\n" + //
    "#.............#\n" + //
    "###############\n";


    private static final String CAN_HEADER = 
    "###############\n" + //
    "#.............#\n" + //
    "#....CAN(t)...#\n" + //
    "#.............#\n" + //
    "###############\n";

    private static final String ERROR_HEADER = 
    "###############\n" + //
    "#.............#\n" + //
    "#....ERRORS...#\n" + //
    "#.............#\n" + //
    "###############\n";

    private static List<Queryable> queryables = new ArrayList<>();

    // public static void startThread() {
    //     new Thread() {
    //         public void run() {
    //           try{
    //           while(!this.isInterrupted() && this.isAlive()){
    //             Thread.sleep(500);
    //             for(int i=0;i<queryables.size(); i++){
    //               queryables.get(i).queryStatus();
    //             }
      
    //             // System.out.println("Updated statuses!");
                
    //           }
    //           }catch(Exception e){
    //               e.printStackTrace();
    //           }
    //         }
    //     }.start();
    // }

    public static void register(Queryable q) {
        queryables.add(q);
    }

    private static void Log(Queryable q, String s){
        System.out.println(q.getName() + " - " + s);
    }

    public static void printReport() {

        List<String> errors = new ArrayList<>();

        // Subsystems header
        System.out.println(REPORTS_HEADER);

        for(int i=0;i< queryables.size();i++){

            Queryable q = queryables.get(i);
            System.out.println("** Subsystem diagnostic report for " + q.getName() + ":");
            Status status = q.diagnosticStatus();

            for(int a=0;a<status.reports.size();a++){
                Report r = status.reports.get(a);
                if(r.reportLevel == ReportLevel.ERROR)
                errors.add(q.getName() + " - " + r.toString());
                Log(q, r.toString());
            }
        }

        
        // CAN header
        System.out.println(CAN_HEADER);

        CANBus canBus = new CANBus(Constants.CANBUS_NAME);
        
        CANBusStatus canInfo = canBus.getStatus();
        
        System.out.println("CANInfo BusOffCount     - " + canInfo.BusOffCount);
        System.out.println("CANInfo BusUtilization  - " + canInfo.BusUtilization);
        System.out.println("CANInfo RX Errors count - " + canInfo.REC);
        System.out.println("CANInfo TX Errors count - " + canInfo.TEC);
        System.out.println("CANInfo Transmit buffer full count - " + canInfo.TxFullCount);
        // Broken turniary operator
        ReportLevel canReportLevel = canInfo.Status.isOK() ? (canInfo.Status.isWarning() ? ReportLevel.WARNING : ReportLevel.ERROR) : ReportLevel.INFO;
        String canStatus = "CAN " + canReportLevel.name() + " - " + canInfo.Status.getName() + " (" + canInfo.Status.getDescription() + ")";
        if(canReportLevel == ReportLevel.ERROR) {
            errors.add(canStatus);
        }
        System.out.println(canStatus);

        // for(int i=0;i<CanDevice.devices.size();i++){

        //     CanDevice device = CanDevice.devices.get(i);
        //     System.out.println("** CAN diagnostic report for " + device.name + ":");
        //     Status status = device.diagnosticStatus();

        //     for(int a=0;a<status.reports.size();a++){
        //         Report r = status.reports.get(a);
        //         if(r.reportLevel == ReportLevel.ERROR)
        //         errors.add(device.getName() + " - " + r.toString());
        //         device.Log(r.toString());
        //     }
        // }

            // System.out.println("Found CAN devices: " + new DeviceFinder().Find());
            
        if(errors.size() > 0) {
            // Errors header
            System.out.println(ERROR_HEADER);
            for(int i=0;i<errors.size(); i++){
                System.out.println(errors.get(i));
            }
        }
    }
}
