// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility.autonomousReplay.nodes;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc4388.utility.autonomousReplay.AutonomousReplayNode;

import java.util.List;

/** Add your docs here. */
public class TalonReplayNode implements AutonomousReplayNode {
    WPI_TalonFX m_Talon;

    TalonReplayNode(WPI_TalonFX talon) {
        m_Talon = talon;
    }

    @Override
    public void startRecording() {
    }

    @Override
    public String stepRecording(double timeStamp, double deltaTime) {
        return m_Talon.get();
    }

    @Override
    public void endRecording() {
    }

    @Override
    public void startPlayback() {
    }

    @Override
    public void stepPlayback(double timeStamp, double deltaTime) {
        
    }

    @Override
    public void endPlayback() {
    }

    @Override
    public String getRecordingData() {
    }

    @Override
    public void setPlaybackData(String data) {
    }
}