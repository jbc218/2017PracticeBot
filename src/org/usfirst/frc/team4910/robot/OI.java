package org.usfirst.frc.team4910.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    public static OI instance = null;
    
    
    public static Joystick rightStick;
    public static Joystick leftStick;

    public OI(){
        rightStick = new Joystick(2);
        leftStick = new Joystick(1); 
    	
    }
    
    
    public static OI getInstance(){
    	return instance == null ? instance = new OI() : instance;
    }
}
