package frc4388.utility.controller;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This is a wrapper for the WPILib Joystick class that represents an XBox 
 * controller.
 * @author frc1675
 */
public class XboxController implements IHandController
{
	public static final int LEFT_X_AXIS = 0;
	public static final int LEFT_Y_AXIS = 1;
	public static final int LEFT_TRIGGER_AXIS = 2;
	public static final int RIGHT_TRIGGER_AXIS = 3;
	public static final int RIGHT_X_AXIS = 4;
	public static final int RIGHT_Y_AXIS = 5;
	public static final int LEFT_RIGHT_DPAD_AXIS = 6;
	public static final int TOP_BOTTOM_DPAD_AXIS = 6;

	public static final int A_BUTTON = 1;
	public static final int B_BUTTON = 2;
	public static final int X_BUTTON = 3;
	public static final int Y_BUTTON = 4;
	public static final int LEFT_BUMPER_BUTTON = 5;
	public static final int RIGHT_BUMPER_BUTTON = 6;
	public static final int BACK_BUTTON = 7;
	public static final int START_BUTTON = 8;

	public static final int LEFT_JOYSTICK_BUTTON = 9;
	public static final int RIGHT_JOYSTICK_BUTTON = 10;

	private static final double LEFT_DPAD_TOLERANCE = -0.9;
	private static final double RIGHT_DPAD_TOLERANCE = 0.9;
	private static final double BOTTOM_DPAD_TOLERANCE = -0.9;
	private static final double TOP_DPAD_TOLERANCE = 0.9;

	private static final double LEFT_TRIGGER_TOLERANCE = 0.5;
	private static final double RIGHT_TRIGGER_TOLERANCE = 0.5;

	private static final double RIGHT_AXIS_UP_TOLERANCE = -0.9;
	private static final double RIGHT_AXIS_DOWN_TOLERANCE = 0.9;
	private static final double RIGHT_AXIS_RIGHT_TOLERANCE = 0.9;
	private static final double RIGHT_AXIS_LEFT_TOLERANCE = -0.9;

	private static final double LEFT_AXIS_UP_TOLERANCE = -0.9;
	private static final double LEFT_AXIS_DOWN_TOLERANCE = 0.9;
	private static final double LEFT_AXIS_RIGHT_TOLERANCE = 0.9;
	private static final double LEFT_AXIS_LEFT_TOLERANCE = -0.9;

	private static final double DEADZONE = 0.1;

	private Joystick m_stick;

	/**
     * Add your docs here.
     */
	public XboxController(int portNumber){
		m_stick = new Joystick(portNumber);
	}

	/**
     * Add your docs here.
     */
	public Joystick getJoyStick() {
		return m_stick;
	}
	
	/**
	 * Checks if the input falls within the deadzone.
	 * @param input from an axis on the controller
	 * @return true if input falls in deadzone, false if input falls outside deadzone
	 */
	private boolean inDeadZone(double input){
		return (Math.abs(input) < DEADZONE);
	}

	/**
	 * Updates an input to have a deadzone around the 0 position
	 * @param input from an axis on the controller
	 * @return updated input
	 */
	private double getAxisWithDeadZoneCheck(double input){
		if(inDeadZone(input)){
			return 0.0;       
		} else {
			return input;
		}
	}

	public boolean getAButton(){
		return m_stick.getRawButton(A_BUTTON);
	}

	public boolean getXButton(){
		return m_stick.getRawButton(X_BUTTON);    
	}

	public boolean getBButton(){
		return m_stick.getRawButton(B_BUTTON);
	}

	public boolean getYButton(){
		return m_stick.getRawButton(Y_BUTTON);
	} 

	public boolean getBackButton(){
		return m_stick.getRawButton(BACK_BUTTON);
	}

	public boolean getStartButton(){
		return m_stick.getRawButton(START_BUTTON);
	} 

	public boolean getLeftBumperButton(){
		return m_stick.getRawButton(LEFT_BUMPER_BUTTON);
	}

	public boolean getRightBumperButton(){
		return m_stick.getRawButton(RIGHT_BUMPER_BUTTON);
	}

	public boolean getLeftJoystickButton(){
		return m_stick.getRawButton(LEFT_JOYSTICK_BUTTON);
	}

	public boolean getRightJoystickButton(){
		return m_stick.getRawButton(RIGHT_JOYSTICK_BUTTON);
	}

	public double getLeftXAxis(){         
		return getAxisWithDeadZoneCheck(m_stick.getRawAxis(LEFT_X_AXIS)); 
	}

	public double getLeftYAxis(){
		return getAxisWithDeadZoneCheck(m_stick.getRawAxis(LEFT_Y_AXIS));
	}

	public double getRightXAxis(){
		return getAxisWithDeadZoneCheck(m_stick.getRawAxis(RIGHT_X_AXIS));
	}

	public double getRightYAxis(){
		return getAxisWithDeadZoneCheck(m_stick.getRawAxis(RIGHT_Y_AXIS)); 
	}

	public double getLeftTriggerAxis(){
		return getAxisWithDeadZoneCheck(m_stick.getRawAxis(LEFT_TRIGGER_AXIS)); 
	}

	public double getRightTriggerAxis(){
		return getAxisWithDeadZoneCheck(m_stick.getRawAxis(RIGHT_TRIGGER_AXIS)); 
	}

	/**
	 * Get the angle input from the dpad.
	 * @return -1 if nothing is pressed, or the angle of the button pressed. 0 = up, 90 = right, etc.
	 */
	public int getDpadAngle() {
		return m_stick.getPOV(0);
	}

	public boolean getDPadLeft(){
      return (m_stick.getRawAxis(LEFT_RIGHT_DPAD_AXIS) < LEFT_DPAD_TOLERANCE);
  	}

  	public boolean getDPadRight(){
      return (m_stick.getRawAxis(LEFT_RIGHT_DPAD_AXIS) > RIGHT_DPAD_TOLERANCE);
  	}

  	public boolean getDPadTop(){
      return (m_stick.getRawAxis(TOP_BOTTOM_DPAD_AXIS) < TOP_DPAD_TOLERANCE);
  	}

  	public boolean getDPadBottom(){
      return (m_stick.getRawAxis(TOP_BOTTOM_DPAD_AXIS) > BOTTOM_DPAD_TOLERANCE);
  	}

	public boolean getLeftTrigger(){
		return (getLeftTriggerAxis() > LEFT_TRIGGER_TOLERANCE);
	}

	public boolean getRightTrigger(){
		return (getRightTriggerAxis() > RIGHT_TRIGGER_TOLERANCE);
	}   

	public boolean getRightAxisUpTrigger(){
		return (getRightYAxis() < RIGHT_AXIS_UP_TOLERANCE);
	}   

	public boolean getRightAxisDownTrigger(){
		return (getRightYAxis() > RIGHT_AXIS_DOWN_TOLERANCE);
	}   

	public boolean getRightAxisLeftTrigger(){
		return (getRightXAxis() > RIGHT_AXIS_LEFT_TOLERANCE);
	}   

	public boolean getRightAxisRightTrigger(){
		return (getRightXAxis() > RIGHT_AXIS_RIGHT_TOLERANCE);
	}   

	public boolean getLeftAxisUpTrigger(){
		return (getLeftYAxis() < LEFT_AXIS_UP_TOLERANCE);
	}   

	public boolean getLeftAxisDownTrigger(){
		return (getLeftYAxis() > LEFT_AXIS_DOWN_TOLERANCE);
	}   

	public boolean getLeftAxisLeftTrigger(){
		return (getLeftXAxis() > LEFT_AXIS_LEFT_TOLERANCE);
	}   

	public boolean getLeftAxisRightTrigger(){
		return (getLeftXAxis() > LEFT_AXIS_RIGHT_TOLERANCE);
	}
}