
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
import org.usfirst.frc.team4910.util.Path;

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
	static Path pat;
	static SendableChooser<String> autoChoose;
	private boolean tunePID=false;
	public static double closeLoopTime=0;
	
    public void robotInit() {
        try{
        	RobotMap.init();
        	oi = OI.getInstance();
        	//drive = DriveTrain.getInstance();
        	drive = DriveTrain.getInstance();
        	pat = new Path();
        	CrashTracker.logRobotInit();
        	iteratorEnabled.register(drive.getLoop());
        	iteratorEnabled.register(sh.getLoop());
        	iteratorEnabled.register(elev.getLoop());
        	iteratorEnabled.register(agi.getLoop());
        	iteratorEnabled.register(vision.getLoop());
        	iteratorDisabled.register(new GyroCalibrator());
        	resetAllSensors();
        	
    		autoChoose = new SendableChooser<String>();
    		autoChoose.addObject("POSITION CHOOSER", "0");
    		autoChoose.addDefault("Do Nothing", "Do Nothing");
    		autoChoose.addObject("Red Left", "Red Left");
    		autoChoose.addObject("Red Middle", "Red Middle");
    		autoChoose.addObject("Red Right", "Red Right");
    		autoChoose.addObject("Blue Left", "Blue Left");
    		autoChoose.addObject("Blue Middle", "Blue Middle");
    		autoChoose.addObject("Blue Right", "Blue Right");
    		SmartDashboard.putData("Auto mode", autoChoose);
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
        	pat.reset();
        	//The point of this is to drive partly to the peg, and then use vision tracking to correct itself,
        	//since we can't expect the human to place it in the right place each time
        	switch((String)autoChoose.getSelected()){
        	case "Do Nothing":
        		break;
        	case "Red Left":
        		pat.register(Path.PathType.Position, 86.94-14.5);
        		pat.register(Path.PathType.Heading, -60.0);
        		pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, ((76.234-14.5)/3.0)-0.5); //Just making sure it doesn't overshoot
        		pat.Iterate();
        		break;
        	case "Red Middle":
        		pat.register(Path.PathType.Position, 100.39/2.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, (100.39/2.0)-0.5);
        		break;
        	case "Red Right":
        		pat.register(Path.PathType.Position, 86.94-14.5);
        		pat.register(Path.PathType.Heading, 60.0);
        		pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, ((76.234-14.5)/3.0)-0.5);
        		pat.Iterate();
        		break;
        	case "Blue Left":
        		pat.register(Path.PathType.Position, 86.94-14.5);
        		pat.register(Path.PathType.Heading, -60.0);
        		pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, ((76.234-14.5)/3.0)-0.5);
        		pat.Iterate();
        		break;
        	case "Blue Middle":
        		pat.register(Path.PathType.Position, 100.39/2.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, (100.39/2.0)-0.5);
        		break;
        	case "Blue Right":
        		pat.register(Path.PathType.Position, 86.94-14.5);
        		pat.register(Path.PathType.Heading, 60.0);
        		pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, (76.234-14.5)/3.0);
        		pat.Iterate();
        		pat.register(Path.PathType.Heading, vision.getCalculatedAngle());
        		pat.register(Path.PathType.Position, ((76.234-14.5)/3.0)-0.5);
        		pat.Iterate();
        		break;
        	default:
        		break;
        	}
        	
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
				if(RobotMap.gearShifter.get().equals(DoubleSolenoid.Value.kForward)){
					RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
					System.out.println("Low gear");
				}else{
					RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
					System.out.println("High gear");
				}
				while(OI.rightStick.getRawButton(2)){}
				
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
    	RobotMap.spig.reset();
    	RobotMap.navxGyro.reset();
    }
    public void writeAllToCSV(){
        RobotMap.writer.writeNext(drive.valString().split("#"), false);
    }
    /**
     * Create CSV file with timestamp for keeping track of values
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
