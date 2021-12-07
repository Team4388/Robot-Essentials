// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility.autonomousReplay.nodes;

import java.util.List;

import frc4388.utility.autonomousReplay.AutonomousReplayNode;

/** Add your docs here. */
public class AutonomousReplayNodeGroup implements AutonomousReplayNode {
    private List<AutonomousReplayNode> replayNodes;

    AutonomousReplayNodeGroup addReplayNode(AutonomousReplayNode node) {
        replayNodes.add(node);
        return this;
    }

    AutonomousReplayNodeGroup removeReplayNode(AutonomousReplayNode node) {
        replayNodes.remove(node);
        return this;
    }

    @Override
    public void startRecording() {
        replayNodes.forEach((node) -> node.startRecording());
    }

    @Override
    public void stepRecording(double timeStamp, double deltaTime) {
        replayNodes.forEach((node) -> node.stepRecording(timeStamp, deltaTime));
    }

    @Override
    public void endRecording() {
        replayNodes.forEach((node) -> node.endRecording());
    }

    @Override
    public void startPlayback() {
        replayNodes.forEach((node) -> node.startPlayback());
    }

    @Override
    public void stepPlayback(double timeStamp, double deltaTime) {
        replayNodes.forEach((node) -> node.stepPlayback(timeStamp, deltaTime));
    }

    @Override
    public void endPlayback() {
        replayNodes.forEach((node) -> node.endPlayback());
    }

    @Override
    public String getRecordingData() {
            
    }

    @Override
    public void setPlaybackData(String data) {
        
    }
}