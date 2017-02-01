package org.usfirst.frc.team4910.util;

public class Util {
	private Util(){}
    /**
     * 
     * @param x
     * @return -1, 0 or 1. This really should have been inside the Math library.
     */
    public static double sign(double x){
        return x>0 ? 1 : (x==0 ? 0 : -1);
    }
}
