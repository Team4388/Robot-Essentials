// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Swerve;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.UtilityStructs.TimedOutput;

public class JoystickPlayback extends Command {
  private final SwerveDrive            swerve;
  private       String                 filename;
  private       int                    mult       = 1;
  private       Scanner                input;
  private final ArrayList<TimedOutput> outputs      = new ArrayList<>();
  private       int                    counter      = 0;
  private       long                   startTime    = 0;
  private       long                   playbackTime = 0;
  private       int                    lastIndex;
  private       boolean                m_finished   = false; // ! find a better way

  /** Creates a new JoystickPlayback. */
  public JoystickPlayback(SwerveDrive swerve, String filename, int mult) {
    this.swerve = swerve;
    this.filename = filename;
    this.mult = mult;

    addRequirements(this.swerve);
  }

  /** Creates a new JoystickPlayback. */
  public JoystickPlayback(SwerveDrive swerve, String filename) {
    this(swerve, filename, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outputs.clear();
    m_finished = false;

    startTime = System.currentTimeMillis();
    playbackTime = 0;
    lastIndex    = 0;
    try {
      input = new Scanner(new File("/home/lvuser/autos/" + filename));

      String line = "";
      while (input.hasNextLine()) {
        line = input.nextLine();

        if (line.isEmpty() || line.isBlank() || line.equals("\n")) {
          continue;
        }
      
        String[] values = line.split(",");
        System.out.println("values: " + values[0] + " " + values[1] + " " + values[2] + " " + values[3]);

        var out = new TimedOutput();
        out.leftX  = Double.parseDouble(values[0]) * mult;
        out.leftY  = Double.parseDouble(values[1]);
        out.rightX = Double.parseDouble(values[2]);
        out.rightY = Double.parseDouble(values[3]);

        out.timedOffset = Long.parseLong(values[4]);

        outputs.add(out);
      }

      input.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter == 0) {
      startTime = System.currentTimeMillis();
      playbackTime = 0;
    } else {
      playbackTime = System.currentTimeMillis() - startTime;
    }

    // skip to reasonable time frame
    // too tired to write comment: ask daniel thomas; it goes to the thing until it's bigger than the other thing
    {
      int i = lastIndex == 0 ? 1 : lastIndex;
      while (i < outputs.size() && outputs.get(i).timedOffset < playbackTime) {
        i++;
      }

      if (i >= outputs.size()) {
        m_finished = true; // ! kind of a hack
        return;
      }
      lastIndex = i;
    }

    TimedOutput lastOut = outputs.get(lastIndex - 1);
    TimedOutput out     = outputs.get(lastIndex);

    double deltaTime     = out.timedOffset - lastOut.timedOffset;
    double playbackDelta = playbackTime    - lastOut.timedOffset;

    double lerpLX = lastOut.leftX  + (out.leftX  - lastOut.leftX)  * (playbackDelta / deltaTime);
    double lerpLY = lastOut.leftY  + (out.leftY  - lastOut.leftY)  * (playbackDelta / deltaTime);
    double lerpRX = lastOut.rightX + (out.rightX - lastOut.rightX) * (playbackDelta / deltaTime);
    double lerpRY = lastOut.rightY + (out.rightY - lastOut.rightY) * (playbackDelta / deltaTime);

    // this.swerve.driveWithInput(new Translation2d(out.leftX,   out.leftY),
    //                              new Translation2d(out.rightX, out.rightY),
    //                              true);
    
    // this.swerve.driveWithInput( new Translation2d(lerpLX, lerpLY),
    //                             new Translation2d(lerpRX, lerpRY),
    //                             true);

    this.swerve.playbackDriveWithInput( new Translation2d(lerpLX, lerpLY),
                                new Translation2d(lerpRX, lerpRY),
                                true);
                             
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    input.close();
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}