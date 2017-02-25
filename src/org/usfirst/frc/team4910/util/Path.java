package org.usfirst.frc.team4910.util;

import java.util.ArrayList;
import java.util.HashMap;

import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.RobotState;
import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveControlState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	private double PosThresh=7.25;
	private Robot robot;
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
		robot = new Robot();
	}
	public void register(PathType pt, double set){
		//pathmap.put(count, path);
		pmap.add(new PathMap(pt, set));
	}
	public void Iterate(){
		double start = Timer.getFPGATimestamp();
		Robot.drive.resetAll();
		for(PathMap current : pmap){
			if(RobotMap.testerCodeEnabled)
				robot.createNewCSV(); //can't make it static because I make a non static call inside there
			double currStart=Timer.getFPGATimestamp();
			switch(current.getPathType()){
			case Heading:


//					double p=0.0,i=0.0,d=0.0;/*IZoneMin=0.0,IZoneMax=0.0;*/
//					p=SmartDashboard.getNumber("kGP", RobotMap.driveGyroPID.getP());
//					i=SmartDashboard.getNumber("kGI", RobotMap.driveGyroPID.getI());
//					d=SmartDashboard.getNumber("kGD", RobotMap.driveGyroPID.getD());
//					//IZoneMin=SmartDashboard.getNumber("kGIZoneMin", RobotMap.driveGyroPID.getIZoneMin());
//					//IZoneMax=SmartDashboard.getNumber("kGIZoneMax", RobotMap.driveGyroPID.getIZoneMax());
//					RobotMap.driveGyroPID.setPID(p, i, d);
					RobotMap.driveGyroPID.resetIntegrator();
					RobotState.resetPosition();
					Robot.drive.setSetpoint(0);
					System.out.println("Turning to "+current.getSetpoint()+" degrees");
					RobotMap.driveGyroPID.resetIntegrator();
					Robot.drive.setControlState(DriveControlState.velocity);
					Timer.delay(.07);
					//Robot.drive.updatePID();
					Robot.drive.setHeadingSetpoint(current.getSetpoint());
					Timer.delay(.07); //test if needed
					RobotMap.driveGyroPID.setMinimumTimeToRun(1.6*Math.abs(current.getSetpoint()/60.0)); //(degrees) / (max degrees / second) (experimentally gained)
					RobotMap.driveGyroPID.setTolerance(current.getSetpoint()/10.0);
					RobotMap.driveGyroPID.setOutputRange(-1.0, 1.0);
					RobotMap.driveGyroPID.setIZoneRange(current.getSetpoint()/10.0, current.getSetpoint()/4.0);
					RobotMap.driveVelocityLeftPID.setOutputRange(-0.5, 0.5);
					RobotMap.driveVelocityRightPID.setOutputRange(-0.5, 0.5);
					
					Timer.delay(.07); //It seems necessary at this point
					while(!RobotMap.driveGyroPID.onTarget()){
						if(!robot.isAutonomous() && RobotMap.isCompBot && !RobotMap.testerCodeEnabled){
							Robot.drive.disableHeadingMode();
							Robot.drive.resetAll();
							System.out.println("Potential error: Path exited early!");
							reset();
							Robot.drive.setControlState(DriveControlState.regular);
							return;
						}
						if(Timer.getFPGATimestamp()-currStart>1.6*Math.abs(current.getSetpoint()/60.0)
								|| (Math.abs(RobotMap.left1.get())<.2 && Math.abs(RobotMap.driveGyroPID.getError())<current.getSetpoint()/10.0 && Timer.getFPGATimestamp()-currStart>0.9*Math.abs(current.getSetpoint()/60.0))){
							//if() statement breaks if it's taken too long or it finishes early
							Timer.delay(.1);
							//System.out.println("End error: "+RobotMap.driveGyroPID.getError()+" Degrees");
							break;
						}
						//System.out.println("Current error: "+RobotMap.driveGyroPID.getError()+" Degrees");
						//Robot.drive.updatePID();
						if(RobotMap.testerCodeEnabled)
							robot.writeAllToCSV();
					}
					//Timer.delay(.1);
					Robot.drive.disableHeadingMode();
					Robot.drive.resetAll();
					System.out.println("Turning took " + (Timer.getFPGATimestamp()-currStart)+" Seconds");
					break;
					
				
			case Position:
				System.out.println("Driving to "+current.getSetpoint()+" inches");
				
				
//				double p=0.0,i=0.0,d=0.0;/*IZoneMin=0.0,IZoneMax=0.0;*/
//				p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
//				i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
//				d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
//				RobotMap.drivePositionLeftPID.setPID(p, i, d);
//				RobotMap.drivePositionRightPID.setPID(p, i, d);
				
				Robot.drive.setControlState(DriveControlState.position);
				Timer.delay(.07);
				Robot.drive.setSetpoints(current.getSetpoint(), current.getSetpoint());
				RobotMap.drivePositionLeftPID.setMinimumTimeToRun(6.0*Math.abs(current.getSetpoint()/RobotMap.leftMaxIPS));
				//(inches) / (max inches / second)
				RobotMap.drivePositionRightPID.setMinimumTimeToRun(6.0*Math.abs(current.getSetpoint()/RobotMap.rightMaxIPS));
				RobotMap.drivePositionLeftPID.setTolerance(1.5);
				RobotMap.drivePositionRightPID.setTolerance(1.5);
				RobotMap.drivePositionLeftPID.setOutputRange(-1.0, 1.0);
				RobotMap.drivePositionRightPID.setOutputRange(-1.0, 1.0);
				RobotMap.drivePositionLeftPID.setIZoneRange(0.0, 10.0);
				RobotMap.drivePositionRightPID.setIZoneRange(0.0, 10.0);
				Timer.delay(.07);
				double startXPos=RobotState.getPosX();
				double startYPos=RobotState.getPosY();
				while(!RobotMap.drivePositionLeftPID.onTarget() || !RobotMap.drivePositionRightPID.onTarget()){
					if(!robot.isAutonomous() && RobotMap.isCompBot && !RobotMap.testerCodeEnabled){
						Robot.drive.disableHeadingMode();
						Robot.drive.resetAll();
						System.out.println("Potential error: Path exited early!");
						reset();
						Robot.drive.setControlState(DriveControlState.regular);
						return;
					}
					if(Timer.getFPGATimestamp()-currStart>PosThresh*Math.abs(current.getSetpoint())/(RobotMap.leftMaxIPS) || OI.leftStick.getRawButton(OI.DisablePIDTester)){
						//loop time > 200% min time
						//Boolean algebra gets tricky sometimes
						//System.out.println("End error: "+RobotMap.drivePositionLeftPID.getError());
						break;
					}
					//System.out.println("Current Error: "+(RobotMap.drivePositionLeftPID.getError()+RobotMap.drivePositionRightPID.getError())/2.0);
        			//Robot.drive.updatePID();
        			if(RobotMap.testerCodeEnabled)
        				robot.writeAllToCSV();
				}
				Robot.drive.setControlState(DriveControlState.regular);
				Robot.drive.resetAll();
				System.out.println("Total error: "+Math.hypot(RobotState.getPosX()-startXPos, RobotState.getPosY()-startYPos));
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
	public synchronized void setPositionTimeThresh(double t){
		PosThresh=t;
	}
	
	
	
	
}
