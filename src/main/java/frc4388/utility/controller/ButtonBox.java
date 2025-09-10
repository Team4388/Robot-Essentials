package frc4388.utility.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBox extends GenericHID {
    public static final int White = 1;
    public static final int One = 2;
    public static final int Two = 3;
    public static final int Three = 4;
    public static final int Four = 5;
    public static final int Five = 6;
    public static final int Six = 7;
    public static final int Seven = 8;
    public static final int Eight = 9;

    public ButtonBox(int ID){
        super(ID);
    }
}
