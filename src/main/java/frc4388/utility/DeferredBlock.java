package frc4388.utility;

import java.util.ArrayList;

// Class for running code snippets whenever the robot is enabled.
public class DeferredBlock {
    private static ArrayList<Runnable> m_blocks_norerun = new ArrayList<>();
    private static ArrayList<Runnable> m_blocks_rerun = new ArrayList<>();
    private static boolean             m_hasRun = false;

    public static void addBlock(Runnable block) {
        addBlock(block, false);
    }


    public static void addBlock(Runnable block, boolean rerun) {
        if(rerun) {
            m_blocks_rerun.add(block);
        } else {
            m_blocks_norerun.add(block);
        }
    }

    public static void execute() {

        // Run blocks that run multiple times.
        for (Runnable block : m_blocks_rerun) {
            block.run();
        }

        // Run blocks that only run once
        if (m_hasRun) return;

        for (Runnable block : m_blocks_norerun) {
            block.run();
        }

        m_blocks_norerun.clear(); // for garbage collection
        m_hasRun = true;
    }
}
