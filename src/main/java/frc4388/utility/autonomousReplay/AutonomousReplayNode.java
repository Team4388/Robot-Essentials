// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility.autonomousReplay;

/** Add your docs here. */
public interface AutonomousReplayNode {
    public void startRecording();
    public void stepRecording(double timeStamp, double deltaTime);
    public void endRecording();
    public void startPlayback();
    public void stepPlayback(double timeStamp, double deltaTime);
    public void endPlayback();
    public String getRecordingData();
    public void setPlaybackData(String data);
}