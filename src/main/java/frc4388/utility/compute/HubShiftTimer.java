
package frc4388.utility.compute;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
public class HubShiftTimer {

  public enum ShiftPhase {
    DISABLED,
    AUTO,
    TRANSITION, // 0 – 10 s
    SHIFT1,     // 10 – 35 s
    SHIFT2,     // 35 – 60 s
    SHIFT3,     // 60 – 85 s
    SHIFT4,     // 85 – 110 s
    ENDGAME     // 110 – 140 s
  }

  public record ShiftInfo(
      ShiftPhase phase,
      double elapsedInShift,
      double remainingInShift,
      boolean isActive) {}
//total teleop time
  public static final double TELEOP_DURATION = 140.0;
//total auto time
  public static final double AUTO_DURATION = 20.0;

//shift start and end times for calculations
  private static final double[] SHIFT_STARTS = {0.0,  10.0, 35.0, 60.0,  85.0, 110.0};
  private static final double[] SHIFT_ENDS   = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

//hub active schedule, true is active and false is inactive
//starts always as active becasue transition is first and is active, but then is inactive for winner or active for loser
  private static final boolean[] WINNER_SCHEDULE = {true,  false, true,  false, true,  true};
  private static final boolean[] LOSER_SCHEDULE  = {true,  true,  false, true,  false, true};

//shift phase names during teleop
  private static final ShiftPhase[] SHIFT_PHASES = {
    ShiftPhase.TRANSITION,
    ShiftPhase.SHIFT1,
    ShiftPhase.SHIFT2,
    ShiftPhase.SHIFT3,
    ShiftPhase.SHIFT4,
    ShiftPhase.ENDGAME
  };

//timer to track time
  private static final Timer teleopTimer = new Timer();
  private static double timerOffset = 0.0;

//fms syncing idk other team did it too
  private static final double RESYNC_THRESHOLD = 3.0;


//call at start of auto to start timer 
  public static void initializeAuto() {
    teleopTimer.restart();
  }
  
//call at start of teleop to start timer again because sometimes delay between auto and telop
  public static void initializeTeleop() {
    timerOffset = 0.0;
    teleopTimer.restart();
  }



//returns the updated shift info based on the winner of auto
  public static ShiftInfo getShiftInfo() {
    if (!DriverStation.isEnabled()) {
      return new ShiftInfo(ShiftPhase.DISABLED, 0.0, 0.0, false);
    }
    if (DriverStation.isAutonomousEnabled()) {
      double autoElapsed = teleopTimer.get(); // timer restarts in initialize()
      return new ShiftInfo(
          ShiftPhase.AUTO,
          autoElapsed,
          Math.max(0.0, AUTO_DURATION - teleopTimer.get()),
          true);
    }
    return computeTeleopShift();
  }

//find auto winner, R = red wins, B = blue wins
  public static Alliance autoWinnerAlliance() {
    String msg = DriverStation.getGameSpecificMessage();
    if (msg != null && msg.length() > 0) {
      char c = msg.charAt(0);
      if (c == 'R') return Alliance.Red;
      if (c == 'B') return Alliance.Blue;
    }
    // backup if no msg, returns auto winner as opposite of our alliance. if we red -> blue wins auto
    Alliance ours = DriverStation.getAlliance().orElse(Alliance.Blue);
    return (ours == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
  }


  //return our schedule for the shifts
  private static boolean[] getSchedule() {
    Alliance ours   = DriverStation.getAlliance().orElse(Alliance.Blue);
    Alliance winner = autoWinnerAlliance();
    return (ours == winner) ? WINNER_SCHEDULE : LOSER_SCHEDULE;
  }

//time since start of teleop
  private static double getTeleopElapsed() {
    double localTime = teleopTimer.get() - timerOffset;

    // Re-sync to FMS time if we drift too far (only when FMS is attached)
    if (DriverStation.isFMSAttached()) {
      double fmsElapsed = TELEOP_DURATION - DriverStation.getMatchTime();
      if (fmsElapsed <= TELEOP_DURATION - 5.0 // ignore the first few seconds of jitter
          && Math.abs(localTime - fmsElapsed) >= RESYNC_THRESHOLD) {
        timerOffset += localTime - fmsElapsed;
        localTime = fmsElapsed;
      }
    }
    return Math.max(0.0, Math.min(TELEOP_DURATION, localTime));
  }

  private static ShiftInfo computeTeleopShift() {
    boolean[] schedule = getSchedule();
    double elapsed = getTeleopElapsed();

    // Find which shift we're in
    int phaseIndex = SHIFT_STARTS.length - 1; // default to last shift if past all bounds
    for (int i = 0; i < SHIFT_STARTS.length; i++) {
      if (elapsed >= SHIFT_STARTS[i] && elapsed < SHIFT_ENDS[i]) {
        phaseIndex = i;
        break;
      }
    }

    double shiftElapsed   = elapsed - SHIFT_STARTS[phaseIndex];
    double shiftRemaining = SHIFT_ENDS[phaseIndex] - elapsed;

    // merge time for elapsed if same active/inactive
    if (phaseIndex > 0 && schedule[phaseIndex] == schedule[phaseIndex - 1]) {
      shiftElapsed = elapsed - SHIFT_STARTS[phaseIndex - 1];
    }

    // merge time for remaining time if same active/inactive status 
    if (phaseIndex < SHIFT_ENDS.length - 1 && schedule[phaseIndex] == schedule[phaseIndex + 1]) {
      shiftRemaining = SHIFT_ENDS[phaseIndex + 1] - elapsed;
    }

    return new ShiftInfo(
        SHIFT_PHASES[phaseIndex],
        shiftElapsed,
        shiftRemaining,
        schedule[phaseIndex]);
  }
}