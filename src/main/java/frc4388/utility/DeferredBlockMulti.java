package frc4388.utility;

import java.util.ArrayList;

public class DeferredBlockMulti {
    private static ArrayList<Runnable> m_blocks = new ArrayList<>();

    public DeferredBlockMulti(Runnable block) {
        m_blocks.add(block);
    }

    public static void execute() {

        for (Runnable block : m_blocks) {
            block.run();
        }
    }
}
