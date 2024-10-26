// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Swerve;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.UtilityStructs.TimedOutput;

public class JoystickRecorder extends Command {
  public  final SwerveDrive            swerve;

  public  final Supplier<Double>       leftX;
  public  final Supplier<Double>       leftY;
  public  final Supplier<Double>       rightX;
  public  final Supplier<Double>       rightY;
  private       String                 filename;
  public  final ArrayList<TimedOutput> outputs   = new ArrayList<>();
  private       long                   startTime = -1;


  /** Creates a new JoystickRecorder. */
  public JoystickRecorder(SwerveDrive swerve, Supplier<Double> leftX,  Supplier<Double> leftY,
                                              Supplier<Double> rightX, Supplier<Double> rightY,
                                              String filename)
  {
    this.swerve = swerve;
    this.leftX  = leftX;
    this.leftY  = leftY;
    this.rightX = rightX;
    this.rightY = rightY;
    this.filename = filename;

    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outputs.clear();

    this.startTime = System.currentTimeMillis();

    outputs.add(new TimedOutput());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var inputs = new TimedOutput();
    inputs.leftX       = leftX.get();
    inputs.leftY       = leftY.get();
    inputs.rightX      = rightX.get();
    inputs.rightY      = rightY.get();
    inputs.timedOffset = System.currentTimeMillis() - startTime;

    outputs.add(inputs);

    swerve.playbackDriveWithInput(new Translation2d(inputs.leftX,  inputs.leftY),
                          new Translation2d(inputs.rightX, inputs.rightY),
                          true);
    
    //System.out.println("RECORDING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    File output = new File("/home/lvuser/autos/" + filename);

    try (PrintWriter writer = new PrintWriter(output)) {
      for (var input : outputs) {
        writer.println( input.leftX  + "," + input.leftY  + "," +
                        input.rightX + "," + input.rightY + "," +
                        input.timedOffset);
      }

      writer.close();
    } catch (IOException e) {
        e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}