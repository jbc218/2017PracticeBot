package org.usfirst.frc.team4910.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * 
 * @author Jason Cobb
 * 
 * Handles operator input
 */
public class OI {
    public static OI instance = null;
    
    
    public static Joystick rightStick;
    public static Joystick leftStick;
    public static Joystick thirdStick;

    public OI(){
        rightStick = new Joystick(2);
        leftStick = new Joystick(1); 
        thirdStick = new Joystick(3);
    	
    }
    
    
    public static OI getInstance(){
    	return instance == null ? instance = new OI() : instance;
    }
}
