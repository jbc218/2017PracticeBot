
package org.usfirst.frc.team4910.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;

import org.usfirst.frc.team4910.iterations.*;
import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveControlState;
import org.usfirst.frc.team4910.subsystems.*;
import org.usfirst.frc.team4910.util.CrashTracker;
import org.usfirst.frc.team4910.util.GyroHelper;

import com.opencsv.CSVWriter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
    
	
	Iterator iteratorEnabled = new Iterator();
	Iterator iteratorDisabled = new Iterator(); //to do functions while idle, implement later
	public static DriveTrain drive;
	static Shooter sh;
	static Elevator elev;
	static Agitator agi;
	static VisionProcessor vision;
	static OI oi;
	private boolean tunePID=false;
	public static double closeLoopTime=0;
	
    public void robotInit() {
        try{
        	RobotMap.init();
        	oi = OI.getInstance();
        	//drive = DriveTrain.getInstance();
        	drive = DriveTrain.getInstance();
        	CrashTracker.logRobotInit();
        	iteratorEnabled.register(drive.getLoop());
        	iteratorEnabled.register(sh.getLoop());
        	iteratorEnabled.register(elev.getLoop());
        	iteratorEnabled.register(agi.getLoop());
        	iteratorEnabled.register(vision.getLoop());
        	iteratorDisabled.register(new GyroCalibrator());
        	resetAllSensors();
        	//RobotMap.g.calibrate();
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void disabledInit(){
        try{
        	CrashTracker.logDisabledInit();
        	iteratorEnabled.stop();
        	iteratorDisabled.start();
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void disabledPeriodic(){
        try{
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void autonomousInit() {
        try{
        	resetAllSensors();
        	CrashTracker.logAutoInit();
        	iteratorEnabled.start();
        	iteratorDisabled.stop();
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }

    public void autonomousPeriodic() {
        try{
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }

    public void teleopInit(){
        try{
        	resetAllSensors();
        	CrashTracker.logTeleopInit();
        	iteratorEnabled.start();
        	iteratorDisabled.stop();
        	System.out.println("Testing");
        	closeLoopTime=0;
        	RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void teleopPeriodic() {
        try{
			if(OI.rightStick.getRawButton(2)){
				while(OI.rightStick.getRawButton(2)){}
				if(RobotMap.gearShifter.get().equals(DoubleSolenoid.Value.kForward)){
					RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
					System.out.println("Reverse");
				}else{
					RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
					System.out.println("Forward");
				}
				
			}
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void testInit(){
        try{
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    
    public void testPeriodic() {
        try{
        	
        }catch(Throwable t){
        	CrashTracker.logThrowableCrash(t);
        	throw t;
        }
    }
    public void resetAllSensors(){
    	RobotMap.left1.setPosition(0);
    	RobotMap.right1.setPosition(0);
    	//RobotMap.g.reset();
    }
    public void writeAllToCSV(){
        RobotMap.writer.writeNext(drive.valString().split("#"), false);
    }
    /**
     * This would be a valid time to make a CSVWriterFactory that uses an iterator variable to loop in the background, but I'm not enterprise enough to do that
     */
    public void createNewCSV(){
    	try{
    		double t=Timer.getFPGATimestamp();
    		Calendar now = Calendar.getInstance();
			String str = String.valueOf(now.get(Calendar.MONTH))+"."+String.valueOf(now.get(Calendar.DAY_OF_MONTH))+"."
					+String.valueOf(now.get(Calendar.HOUR_OF_DAY))+"."+String.valueOf(now.get(Calendar.MINUTE))+"."+String.valueOf(now.get(Calendar.SECOND));
			File f = new File("/home/lvuser/TestingData"+str+".csv");
			RobotMap.writer = new CSVWriter(new FileWriter(f), ',');
//			String[] tabNames = ("Time#LeftError#RightError#LeftPosition#RightPosition#LeftVelocity#RightVelocity#LeftSetpoint"
//					+ "#RightSetpoint#WeightedLeftError#WeightedRightError#WeightedLeftPosition#WeightedRightPosition"
//					+ "#WeightedLeftVelocity#WeightedRightVelocity#WeightedLeftSetpoint#WeightedRightSetpoint#kP#kI#kD#kFL#kFR#kV#kGP#kGI#kGD#Voltage#Heading#HeadingSetpoint").split("#");
			String[] tabNames = drive.keyString().split("#");
			Timer.delay(.01);
			RobotMap.writer.writeNext(tabNames, true);
			System.out.println("Created new CSV file with name: TestingData"+str+".csv in "+(Timer.getFPGATimestamp()-t)+" Seconds");
    	}catch(IOException e){
    		e.printStackTrace();
    	}
    }
}
