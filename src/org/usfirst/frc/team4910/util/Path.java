package org.usfirst.frc.team4910.util;

import java.util.ArrayList;
import java.util.HashMap;

import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
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
	private double PosThresh=7.5;
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
				if(false){
				//if(current.getSetpoint()>50.0){
	//				double p=0.0,i=0.0,d=0.0;/*IZoneMin=0.0,IZoneMax=0.0;*/
	//				p=SmartDashboard.getNumber("kGP", RobotMap.driveGyroPID.getP());
	//		    	i=SmartDashboard.getNumber("kGI", RobotMap.driveGyroPID.getI());
	//		    	d=SmartDashboard.getNumber("kGD", RobotMap.driveGyroPID.getD());
	//		    	IZoneMin=SmartDashboard.getNumber("kGIZoneMin", RobotMap.driveGyroPID.getIZoneMin());
	//		    	IZoneMax=SmartDashboard.getNumber("kGIZoneMax", RobotMap.driveGyroPID.getIZoneMax());
			    	RobotMap.driveGyroPID.setPID(RobotMap.GyroKp, RobotMap.GyroKi, RobotMap.GyroKd);
					RobotMap.driveGyroPID.resetIntegrator();
					RobotMap.drivePositionLeftPID.resetIntegrator();
					RobotMap.drivePositionRightPID.resetIntegrator();
					RobotMap.driveVelocityLeftPID.resetIntegrator();
					RobotMap.driveVelocityRightPID.resetIntegrator();
					RobotMap.left1.setEncPosition(0);
					RobotMap.right1.setEncPosition(0);
					Robot.drive.setSetpoint(0);
					System.out.println("Turning to "+current.getSetpoint()+" degrees");
					RobotMap.driveGyroPID.resetIntegrator();
					RobotMap.drivePositionLeftPID.resetIntegrator();
					RobotMap.drivePositionRightPID.resetIntegrator();
					RobotMap.driveVelocityLeftPID.resetIntegrator();
					RobotMap.driveVelocityRightPID.resetIntegrator();
					Robot.drive.setControlState(DriveControlState.velocity);
					Timer.delay(.07);
					//Robot.drive.updatePID();
	        		Robot.drive.setHeadingSetpoint(current.getSetpoint());
	        		Timer.delay(.07); //test if needed
	        		RobotMap.driveGyroPID.setMinimumTimeToRun(2.2*Math.abs(current.getSetpoint()/60.0)); //(degrees) / (max degrees / second) (experimentally gained)
	        		RobotMap.driveGyroPID.setTolerance(2);
	        		RobotMap.driveGyroPID.setOutputRange(-1.0, 1.0);
	        		RobotMap.driveGyroPID.setIZoneRange(2.0, current.getSetpoint()/4.0);
	        		RobotMap.driveVelocityLeftPID.setOutputRange(-0.5, 0.5);
	        		RobotMap.driveVelocityRightPID.setOutputRange(-0.5, 0.5);
	        		
	        		Timer.delay(.07); //It seems necessary at this point
	        		while(!RobotMap.driveGyroPID.onTarget()){
	        			if(Timer.getFPGATimestamp()-currStart>2.8*Math.abs(current.getSetpoint()/60.0)
	        					|| (Math.abs(RobotMap.left1.get())<.1 && Math.abs(RobotMap.driveGyroPID.getError())<2 && Timer.getFPGATimestamp()-currStart>1.8*Math.abs(current.getSetpoint()/60.0))){
	        				System.out.println("End error: "+RobotMap.driveGyroPID.getError()+" Degrees");
	        				break;
	        			}
	        			System.out.println("Current error: "+RobotMap.driveGyroPID.getError()+" Degrees");
	        			//Robot.drive.updatePID();
	        			robot.writeAllToCSV();
	        		}
	        		Robot.drive.disableHeadingMode();
	        		Robot.drive.resetAll();
	        		System.out.println("Turning took " + (Timer.getFPGATimestamp()-currStart)+" Seconds");
					break;
				}else{ //for setpoints less than 50
					   //I shouldn't need this, but I can't afford to waste more time

					double p=0.0,i=0.0,d=0.0;/*IZoneMin=0.0,IZoneMax=0.0;*/
					p=SmartDashboard.getNumber("kGP", RobotMap.driveGyroPID.getP());
					i=SmartDashboard.getNumber("kGI", RobotMap.driveGyroPID.getI());
					d=SmartDashboard.getNumber("kGD", RobotMap.driveGyroPID.getD());
					//IZoneMin=SmartDashboard.getNumber("kGIZoneMin", RobotMap.driveGyroPID.getIZoneMin());
					//IZoneMax=SmartDashboard.getNumber("kGIZoneMax", RobotMap.driveGyroPID.getIZoneMax());
					RobotMap.driveGyroPID.setPID(p, i, d);
					RobotMap.driveGyroPID.resetIntegrator();
					RobotMap.drivePositionLeftPID.resetIntegrator();
					RobotMap.drivePositionRightPID.resetIntegrator();
					RobotMap.driveVelocityLeftPID.resetIntegrator();
					RobotMap.driveVelocityRightPID.resetIntegrator();
					RobotMap.left1.setEncPosition(0);
					RobotMap.right1.setEncPosition(0);
					Robot.drive.setSetpoint(0);
					System.out.println("Turning to "+current.getSetpoint()+" degrees");
					RobotMap.driveGyroPID.resetIntegrator();
					RobotMap.drivePositionLeftPID.resetIntegrator();
					RobotMap.drivePositionRightPID.resetIntegrator();
					RobotMap.driveVelocityLeftPID.resetIntegrator();
					RobotMap.driveVelocityRightPID.resetIntegrator();
					Robot.drive.setControlState(DriveControlState.velocity);
					Timer.delay(.07);
					//Robot.drive.updatePID();
					Robot.drive.setHeadingSetpoint(current.getSetpoint());
					Timer.delay(.07); //test if needed
					RobotMap.driveGyroPID.setMinimumTimeToRun(3.0*Math.abs(current.getSetpoint()/60.0)); //(degrees) / (max degrees / second) (experimentally gained)
					RobotMap.driveGyroPID.setTolerance(current.getSetpoint()/10.0);
					RobotMap.driveGyroPID.setOutputRange(-1.0, 1.0);
					RobotMap.driveGyroPID.setIZoneRange(current.getSetpoint()/10.0, current.getSetpoint()/4.0);
					RobotMap.driveVelocityLeftPID.setOutputRange(-0.5, 0.5);
					RobotMap.driveVelocityRightPID.setOutputRange(-0.5, 0.5);

					Timer.delay(.07); //It seems necessary at this point
					while(!RobotMap.driveGyroPID.onTarget()){
						if(Timer.getFPGATimestamp()-currStart>3.6*Math.abs(current.getSetpoint()/60.0)
								|| (Math.abs(RobotMap.left1.get())<.08 && Math.abs(RobotMap.driveGyroPID.getError())<current.getSetpoint()/10.0 && Timer.getFPGATimestamp()-currStart>2.8*Math.abs(current.getSetpoint()/60.0))){
							//if() statement breaks if it's taken too long or it finishes early
							Timer.delay(.1);
							System.out.println("End error: "+RobotMap.driveGyroPID.getError()+" Degrees");
							break;
						}
						System.out.println("Current error: "+RobotMap.driveGyroPID.getError()+" Degrees");
						//Robot.drive.updatePID();
						robot.writeAllToCSV();
					}
					//Timer.delay(.1);
					Robot.drive.disableHeadingMode();
					Robot.drive.resetAll();
					System.out.println("Turning took " + (Timer.getFPGATimestamp()-currStart)+" Seconds");
					break;
					
				}
			case Position:
				System.out.println("Driving to "+current.getSetpoint()+" inches");
				RobotMap.driveGyroPID.resetIntegrator();
				RobotMap.drivePositionLeftPID.resetIntegrator();
				RobotMap.drivePositionRightPID.resetIntegrator();
				RobotMap.driveVelocityLeftPID.resetIntegrator();
				RobotMap.driveVelocityRightPID.resetIntegrator();
				Robot.drive.setControlState(DriveControlState.position);
				Timer.delay(.07);
				Robot.drive.setSetpoints(current.getSetpoint(), current.getSetpoint());
				RobotMap.drivePositionLeftPID.setMinimumTimeToRun(6.0*Math.abs(current.getSetpoint()/RobotMap.leftMaxIPS));
				//(inches) / (max inches / second)
				RobotMap.drivePositionRightPID.setMinimumTimeToRun(6.0*Math.abs(current.getSetpoint()/RobotMap.rightMaxIPS));
				RobotMap.drivePositionLeftPID.setTolerance(1.5);
				RobotMap.drivePositionRightPID.setTolerance(1.5);
				RobotMap.drivePositionLeftPID.setOutputRange(-0.5, 0.5);
				RobotMap.drivePositionRightPID.setOutputRange(-0.5, 0.5);
				RobotMap.drivePositionLeftPID.setIZoneRange(0.0, 10.0);
				RobotMap.drivePositionRightPID.setIZoneRange(0.0, 10.0);
				Timer.delay(.07);
				while(!RobotMap.drivePositionLeftPID.onTarget() || !RobotMap.drivePositionRightPID.onTarget()){
					if(Timer.getFPGATimestamp()-currStart>PosThresh*Math.abs(current.getSetpoint())/(RobotMap.leftMaxIPS) || OI.leftStick.getRawButton(OI.DisablePIDTester)){
						//loop time > 200% min time
						//Boolean algebra gets tricky sometimes
						System.out.println("End error: "+RobotMap.drivePositionLeftPID.getError());
						break;
					}
					System.out.println("Current Error: "+(RobotMap.drivePositionLeftPID.getError()+RobotMap.drivePositionRightPID.getError())/2.0);
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
	public synchronized void setPositionTimeThresh(double t){
		PosThresh=t;
	}
	
	
	
	
}
