package org.usfirst.frc.team4910.util;

import java.util.ArrayList;
import java.util.HashMap;

import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveControlState;

import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * @author Jason Cobb
 * Purpose is to register and sequence setpoints
 */
public class Path {
	public enum PathType{
		Heading, Position;
	}
	//private HashMap<Integer, HashMap<PathType, Double>> pathmap = new HashMap<Integer, HashMap<PathType, Double>>();
	private ArrayList<PathMap> pmap;
	private class PathMap{
		private PathType pathType;
		private double setpoint;
		public PathMap(PathType pt, double set){
			pathType=pt;
			setpoint=set;
		}
		public PathType getPathType(){
			return pathType;
		}
		public double getSetpoint(){
			return setpoint;
		}
	}
	public Path(){
		pmap = new ArrayList<PathMap>();
	}
	public void register(PathType pt, double set){
		//pathmap.put(count, path);
		pmap.add(new PathMap(pt, set));
	}
	public void Iterate(){
		double start = Timer.getFPGATimestamp();
		Robot robot = new Robot();
		Robot.drive.resetAll();
		for(PathMap current : pmap){
			robot.createNewCSV(); //can't make it static because I make a non static call inside there
			double currStart=Timer.getFPGATimestamp();
			switch(current.getPathType()){
			case Heading:
				Robot.drive.setSetpoint(0);
				System.out.println("Turning to "+current.getSetpoint()+" degrees");
				Robot.drive.setControlState(DriveControlState.velocity);
				Timer.delay(.07);
				//Robot.drive.updatePID();
        		Robot.drive.setHeadingSetpoint(current.getSetpoint());
        		Timer.delay(.07); //test if needed
        		RobotMap.driveGyroPID.setMinimumTimeToRun(3.25*Math.abs(current.getSetpoint()/60.0)); //(degrees) / (max degrees / second)
        		RobotMap.driveGyroPID.setTolerance(4);
        		Timer.delay(.07); //It seems necessary at this point
        		while(!RobotMap.driveGyroPID.onTarget()){
        			if(Timer.getFPGATimestamp()-currStart>3.25*Math.abs(current.getSetpoint()/60.0)){
        				System.out.println("End error: "+RobotMap.driveGyroPID.getError()+" Degrees");
        				break;
        			}
        			//Robot.drive.updatePID();
        			robot.writeAllToCSV();
        		}
        		Robot.drive.disableHeadingMode();
        		Robot.drive.resetAll();
        		System.out.println("Turning took " + (Timer.getFPGATimestamp()-currStart)+" Seconds");
				break;
			case Position:
				System.out.println("Driving to "+current.getSetpoint()+" inches");
				Robot.drive.setControlState(DriveControlState.position);
				Timer.delay(.07);
				Robot.drive.setSetpoints(current.getSetpoint(), current.getSetpoint());
				RobotMap.drivePositionLeftPID.setMinimumTimeToRun(1.5*Math.abs(current.getSetpoint()/RobotMap.leftMaxIPS));
				//(inches) / (max inches / second)
				RobotMap.drivePositionRightPID.setMinimumTimeToRun(1.5*Math.abs(current.getSetpoint()/RobotMap.rightMaxIPS));
				RobotMap.drivePositionLeftPID.setTolerance(1.5);
				RobotMap.drivePositionRightPID.setTolerance(1.5);
				RobotMap.drivePositionLeftPID.setOutputRange(-0.5, 0.5);
				RobotMap.drivePositionRightPID.setOutputRange(-0.5, 0.5);
				RobotMap.drivePositionLeftPID.setIZoneRange(0.0, 10.0);
				RobotMap.drivePositionRightPID.setIZoneRange(0.0, 10.0);
				Timer.delay(.07);
				while(!RobotMap.drivePositionLeftPID.onTarget() || !RobotMap.drivePositionRightPID.onTarget()){
					if(Timer.getFPGATimestamp()-currStart>15.0*Math.abs(current.getSetpoint())/(RobotMap.leftMaxIPS/2.0) || OI.leftStick.getRawButton(OI.DisablePIDTester)){
						//loop time > 200% min time
						//Boolean algebra gets tricky sometimes
						System.out.println("End error: "+RobotMap.drivePositionLeftPID.getError());
						break;
					}
        			//Robot.drive.updatePID();
        			robot.writeAllToCSV();
				}
				Robot.drive.resetAll();
				System.out.println("Driving took " + (Timer.getFPGATimestamp()-currStart)+" Seconds");
				break;
			}
			Timer.delay(.07);
		}
		System.out.println("Path overall time was "+(Timer.getFPGATimestamp()-start) + " Seconds");
		reset();
		Robot.drive.setControlState(DriveControlState.regular);
	}
	/**
	 * This clears all path elements. This shouldn't need to be called externally, but it's there just in case.
	 */
	public synchronized void reset(){
		pmap.clear();
	}
	
	
	
	
}
