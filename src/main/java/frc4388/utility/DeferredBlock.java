package frc4388.utility;

import java.util.ArrayList;

public class DeferredBlock {
    private static ArrayList<Runnable> m_blocks = new ArrayList<>();
    private static boolean             m_hasRun = false;

    public DeferredBlock(Runnable block) {
        m_blocks.add(block);
    }

    public static void execute() {
        if (m_hasRun) return;

        for (Runnable block : m_blocks) {
            block.run();
        }

        m_blocks.clear(); // for garbage collection
        m_hasRun = true;
    }
}
