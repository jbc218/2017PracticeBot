package org.usfirst.frc.team4910.subsystems;

import java.io.File;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.RobotState;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveControlState;
import org.usfirst.frc.team4910.util.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveTrain {
	
	public enum DriveControlState{
		regular, position, velocity, motionprofile;
	}
	private DriveControlState wantedState;
	private double setpointLeft=0;
	private double setpointRight=0;
	private double setpointHeading=0;
	private double closeLoopTime=0.0;
	private double leftOut=0.0,rightOut=0.0;
	private static DriveTrain instance;
	private DriveControlState currentState=DriveControlState.regular;
	public boolean hasIterated=false;
	private boolean headingMode=false;
	private double last_world_linear_accel_x;
	private double last_world_linear_accel_y;
	public static double kG=SmartDashboard.getNumber("current Kg", 0.0);
	private Map<String, Double> map = Collections.synchronizedMap(new HashMap<String, Double>());
	private Set<Map.Entry<String, Double>> Values = map.entrySet();
	private double currentStateStartTime=0.0;
	private double maxAccel=4.708244561906604; //m/s/s
	private double maxJerk=2.00025175417449; //m/s/s/s
	private double maxVelocity=0.07436226410167242; //m/s
	//TODO: test out trajectory, calculate max acceleration and max jerk
	private Trajectory.Config conf = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, .02, maxVelocity, maxAccel , maxJerk );
	private Waypoint[] points;
	private TankModifier modifier;
	private Trajectory leftTrajectory;
	private Trajectory rightTrajectory;
	private Trajectory trajectory;
	private EncoderFollower leftEF, rightEF;
	private double leftTrajectoryOutput=0.0, rightTrajectoryOutput=0.0;
	private boolean trajectoryMode=false;
	private ContinuousAngleTracker fusedAngle = new ContinuousAngleTracker();
	private boolean reverse=false;
	private double posX=0.0, posY=0.0, posXNavX=0.0, posYNavX=0.0, lastLeftEncPos=0.0, lastRightEncPos=0.0;
	private final Iterate iter = new Iterate(){
        private boolean stateChanged;
        
		@Override
		public void init() {
			//updatePID();
			RobotMap.navxGyro.reset();
			fusedAngle.setAngleAdjustment(RobotMap.navxGyro.getFusedHeading());
			setControlState(DriveControlState.regular);
			resetAll();
			posX=0.0;
			posY=0.0;
			posXNavX=0.0;
			posYNavX=0.0;
			lastLeftEncPos=0.0;
			lastRightEncPos=0.0;
			hasIterated=false;
		}

		@Override
		public void exec() {
			synchronized(DriveTrain.this){
				double now = Timer.getFPGATimestamp();
				if(OI.leftStick.getRawButton(OI.ReverseDrive)){
					while(OI.leftStick.getRawButton(OI.ReverseDrive));
					reverse=!reverse;
				}
				fusedAngle.nextAngle(RobotMap.navxGyro.getFusedHeading());
//				double curr_world_linear_accel_x = RobotMap.RIOAccel.getX();
//	        	double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
//		    	last_world_linear_accel_x = curr_world_linear_accel_x;
//		    	double curr_world_linear_accel_y = RobotMap.RIOAccel.getY();
//		    	double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
//		    	last_world_linear_accel_y = curr_world_linear_accel_y;
//		    	double jerkXY = Math.hypot(currentJerkX,currentJerkY); //net jerk in XY plane
//		    	double accelXY = Math.hypot(curr_world_linear_accel_x,curr_world_linear_accel_y);
//		    	maxAccel = Math.max(maxAccel, accelXY);
//		    	maxJerk = Math.max(maxJerk, jerkXY);
//		    	maxVelocity = Math.max(maxVelocity, Math.hypot(
//		    			0.0254*RobotState.getLeftSpeed(),0.0254*RobotState.getRightSpeed() ));
//		    	SmartDashboard.putNumber("InternalAccelX", curr_world_linear_accel_x);
//		    	SmartDashboard.putNumber("InternalAccelY", curr_world_linear_accel_y);
//		    	SmartDashboard.putNumber("InternalJerkX", currentJerkX);
//		    	SmartDashboard.putNumber("InternalJerkY", currentJerkY);
				DriveControlState newState;
				switch(currentState){
				case regular:
					newState = handleRegular();
					break;
				case position:
					newState = handlePosition();
					break;
				case velocity:
					newState = handleVelocity();
					break;
				case motionprofile:
					newState = handleMP();
					break;
				default:
					newState = DriveControlState.regular;
					break;
				}
				if(newState!=currentState){
					System.out.println("Drive state "+currentState+" To "+newState);
					currentState=newState;
					currentStateStartTime=now;
					stateChanged=true;
				}
				outputToDashboard(now);
				hasIterated=true;
				
			}
		}

		@Override
		public void end() {
			System.out.println("Max velocity: "+maxVelocity+"\nMax accel: "+maxAccel+"\nMax jerk: "+maxJerk);
			resetAll();
		}
		
	};
	
	private DriveTrain(){
		//updatePID();
	}
	
	public static DriveTrain getInstance(){
		return instance==null ? instance = new DriveTrain() : instance;
	}
	
	public Iterate getLoop(){
		return iter;
	}
	public synchronized boolean isInHeadingMode(){
		return headingMode;
	}
	private synchronized DriveControlState handleRegular(){
		setpointLeft=-OI.leftStick.getY();
		setpointRight=-OI.rightStick.getY();
		setpointLeft = reverse && !headingMode ? OI.rightStick.getY() : setpointLeft;
		setpointRight = reverse && !headingMode ? OI.leftStick.getY() : setpointRight;
		double G=0.0;
		if(headingMode){
			G=RobotMap.driveGyroPID.calculate(RobotMap.spig.getAngle());
		}
		drive(setpointLeft-G,setpointRight+G);
		switch(wantedState){
		case regular:
			return DriveControlState.regular;
		case position:
			resetAll();
			return DriveControlState.position;
		case velocity:
			resetAll();
			return DriveControlState.velocity;
		case motionprofile:
			return DriveControlState.motionprofile;
		default:
			return DriveControlState.regular;
		}
	}
	private synchronized DriveControlState handlePosition(){
		drive(RobotMap.drivePositionLeftPID.calculate(RobotState.getLeftPos()), 
				RobotMap.drivePositionRightPID.calculate(RobotState.getRightPos()));
//		if(trajectoryMode){
//			double dAngle = Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftEF.getHeading())-RobotMap.spig.getAngle());
//			double turn = 0.8*(-1/80.0)*dAngle;
//			double lc=leftEF.calculate(-RobotMap.left1.getEncPosition());
//			double rc=rightEF.calculate(RobotMap.right1.getEncPosition());
//			leftTrajectoryOutput = lc+turn;
//			rightTrajectoryOutput = rc-turn;
//			SmartDashboard.putNumber("Left Trajectory Output", leftTrajectoryOutput);
//			SmartDashboard.putNumber("Right Trajectory Output", rightTrajectoryOutput);
//		}
		switch(wantedState){
		case regular:
			resetAll();
			return DriveControlState.regular;
		case position:
			return DriveControlState.position;
		case velocity:
			resetAll();
			return DriveControlState.velocity;
		case motionprofile:
			return DriveControlState.motionprofile;
		default:
			return DriveControlState.regular;
		}
	}
	private synchronized DriveControlState handleVelocity(){
		if(headingMode){
			double G=RobotMap.driveGyroPID.calculate(RobotMap.spig.getAngle());
			System.out.println("Gyro output: "+G);
			RobotMap.driveVelocityLeftPID.setSetpoint((setpointLeft-RobotMap.leftMaxIPS*G));
			RobotMap.driveVelocityRightPID.setSetpoint((setpointRight+RobotMap.rightMaxIPS*G));
		}
		drive(RobotMap.driveVelocityLeftPID.calculate(RobotState.getLeftSpeed()),
				RobotMap.driveVelocityRightPID.calculate(RobotState.getRightSpeed()));
//		Timer.delay(.1);
		switch(wantedState){
		case regular:
			resetAll();
			return DriveControlState.regular;
		case position:
			resetAll();
			return DriveControlState.position;
		case velocity:
			return DriveControlState.velocity;
		case motionprofile:
			return DriveControlState.motionprofile;
		default:
			return DriveControlState.regular;
		}
		
	}
	private synchronized DriveControlState handleMP(){
		//TODO: unimplement the rest of this
		
		return DriveControlState.motionprofile;
	}
	public synchronized void setControlState(DriveControlState dcs){
		wantedState=dcs;
	}
	/**
	 * 
	 * @param set either a gyro heading, velocity, distance, or percent
	 */
	public synchronized void setSetpoint(double set){
		setSetpoints(set,set);
	}
	public synchronized void setSetpoints(double left, double right){
		closeLoopTime=Timer.getFPGATimestamp();
		setpointLeft=left;
		setpointRight=right;
		if(currentState==DriveControlState.velocity && !headingMode){
			RobotMap.driveVelocityLeftPID.setSetpoint(setpointLeft);
			RobotMap.driveVelocityRightPID.setSetpoint(setpointRight);
		}else if(currentState==DriveControlState.position && !headingMode){
			RobotMap.drivePositionLeftPID.setSetpoint(setpointLeft);
			RobotMap.drivePositionRightPID.setSetpoint(setpointRight);
			RobotMap.drivePositionLeftPID.resetIntegrator();
			RobotMap.drivePositionRightPID.resetIntegrator();
//			points = new Waypoint[]{new Waypoint(0,0,0),new Waypoint(left*0.0254, right*0.0254,0)};
//			trajectory = Pathfinder.generate(points, conf);
//			modifier = new TankModifier(trajectory).modify(25.375*0.0254);
//			leftTrajectory = modifier.getLeftTrajectory();
//			rightTrajectory = modifier.getRightTrajectory();
//			leftEF = new EncoderFollower(leftTrajectory);
//			rightEF = new EncoderFollower(rightTrajectory);
//			leftEF.configureEncoder(RobotMap.left1.getEncPosition(), (int) RobotMap.EncCountsPerRev, 0.0254*RobotMap.DriveWheelDiameter);
//			rightEF.configureEncoder(RobotMap.right1.getEncPosition(), (int) RobotMap.EncCountsPerRev, 0.0254*RobotMap.DriveWheelDiameter);
//			leftEF.configurePIDVA(RobotMap.drivePositionLeftPID.getP(), RobotMap.drivePositionLeftPID.getI(), RobotMap.drivePositionLeftPID.getD(), 1/maxVelocity, maxAccel/2);
//			rightEF.configurePIDVA(RobotMap.drivePositionRightPID.getP(), RobotMap.drivePositionRightPID.getI(), RobotMap.drivePositionRightPID.getD(), 1/maxVelocity, maxAccel/2);
//			Pathfinder.writeToCSV(new File("/home/lvuser/LeftTrajectory.csv"), leftTrajectory);
//			Pathfinder.writeToCSV(new File("/home/lvuser/RightTrajectory.csv"), rightTrajectory);
//			trajectoryMode=true;
		}
	}
	public synchronized void setHeadingSetpoint(double set){
		setpointHeading=set;
		RobotMap.driveGyroPID.setSetpoint(set);
		headingMode=true;
	}
	public synchronized void disableHeadingMode(){
		headingMode=false;
		RobotMap.spig.reset();
		resetAll();
	}
	public synchronized double[] getSetpoints(){
		return new double[]{setpointLeft,setpointRight};
	}
	private synchronized void drive(double left, double right){
		leftOut=-left;
		rightOut=right;
        if ( Math.abs(left) < .075 && !headingMode && currentState==DriveControlState.regular) {
        	RobotMap.left1.set(0);
        }else{
        	RobotMap.left1.set(-left);
        }
        if ( Math.abs(right) < .075 && !headingMode && currentState==DriveControlState.regular) {
        	RobotMap.right1.set(0);
        }else{
        	RobotMap.right1.set(right);
        }
	}
	public synchronized void resetAll(){
		fusedAngle.reset();
		RobotMap.spig.reset();
		RobotMap.left1.setPosition(0);
		RobotMap.right1.setPosition(0);
		RobotMap.drivePositionLeftPID.reset();
		RobotMap.drivePositionRightPID.reset();
		RobotMap.driveVelocityLeftPID.reset();
		RobotMap.driveVelocityRightPID.reset();
		RobotMap.driveGyroPID.reset();
		headingMode=false;
	}
	
	public synchronized void outputToDashboard(double time){
		updateHashTable();
		synchronized(map){
			for(Map.Entry<String, Double> me : map.entrySet()){
				SmartDashboard.putNumber(me.getKey(), me.getValue());
			}
		}
        SmartDashboard.putNumber("SPI gyro center", RobotMap.spig.getCenter());
        SmartDashboard.putNumber("heading error", setpointHeading-RobotMap.spig.getAngle());
        SmartDashboard.putNumber("battery voltage", DriverStation.getInstance().getBatteryVoltage());
        SmartDashboard.putNumber("Loop time", time);

        
        SmartDashboard.putBoolean("Has collided", ( Math.abs(RobotState.getJerkX()) > .7f ) ||
                ( Math.abs(RobotState.getJerkY()) > .7f));
	}
	public DriveControlState getCurrentState(){
		return currentState;
	}
	
    public static double rotationsToInches(double rotations) {
        return rotations * (RobotMap.DriveWheelDiameter * Math.PI);
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (RobotMap.DriveWheelDiameter * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inchespersecond) {
        return inchesToRotations(inchespersecond) * 60;
    }
    public static double countsToInches(double counts){
    	return rotationsToInches(counts/RobotMap.EncCountsPerRev);
    }
    public static double inchesToCounts(double cps){
    	return inchesToRotations(cps)*RobotMap.EncCountsPerRev;
    }
    /**
     * Safe way of calling every single element of the map without incomplete blocks and overall ease
     * @return list of all v in (k, v) with the delimiter "#"
     */
	public synchronized String valString(){
    	synchronized(map){
    		Iterator<Map.Entry<String, Double>> i = Values.iterator();
    		String s="";
    		while(i.hasNext()){
    			Map.Entry<String, Double> me = (Map.Entry<String, Double>) i.next();
    			s = s.concat(me.getValue().toString()).concat("#");
    		}
    		return s.substring(0, s.length()-1); //Yes, I'm allowed to return from inside a sync block. "map" will be released correctly. And, I'm just removing the "#" at the end.
    	}
    }
	
	public synchronized String keyString(){
    	synchronized(map){
    		Iterator<Map.Entry<String, Double>> i = Values.iterator();
    		String s="";
    		while(i.hasNext()){
    			Map.Entry<String, Double> me = (Map.Entry<String, Double>) i.next();
    			s = s.concat(me.getKey().toString()).concat("#");
    		}
    		return s.substring(0, s.length()-1);
    	}
	}
	
	
	private synchronized void updateHashTable(){
		synchronized(map){
	        double errL=0,errR=0, p=0,i=0,d=0,f1=0,f2=0,outputMin=-1.0, outputMax=1.0, accumL=0.0, accumR=0.0, accumG=0.0, IZoneMax=Double.MAX_VALUE, IZoneMin=0.0;
	        if(currentState==DriveControlState.position){
	        	errL=RobotMap.drivePositionLeftPID.getError();
	        	errR=RobotMap.drivePositionRightPID.getError();
				p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
	        	i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
	        	d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
	        	f1=SmartDashboard.getNumber("kFL", RobotMap.drivePositionLeftPID.getF());
	        	f2=SmartDashboard.getNumber("kFR", RobotMap.drivePositionRightPID.getF());
	        	IZoneMin=SmartDashboard.getNumber("IZoneMin", RobotMap.drivePositionLeftPID.getIZoneMin());
	        	IZoneMax=SmartDashboard.getNumber("IZoneMax", RobotMap.drivePositionLeftPID.getIZoneMax());
	        	
	        	accumL=RobotMap.drivePositionLeftPID.getErrorSum();
	        	accumR=RobotMap.drivePositionRightPID.getErrorSum();
	        	outputMin=SmartDashboard.getNumber("outputMin", RobotMap.drivePositionLeftPID.getMinOut());
	        	outputMax=SmartDashboard.getNumber("outputMax", RobotMap.drivePositionLeftPID.getMaxOut());
	        }else if(currentState==DriveControlState.velocity){
	        	errL=RobotMap.driveVelocityLeftPID.getError();
	        	errR=RobotMap.driveVelocityRightPID.getError();
				p=SmartDashboard.getNumber("kP", RobotMap.driveVelocityLeftPID.getP());
	        	i=SmartDashboard.getNumber("kI", RobotMap.driveVelocityLeftPID.getI());
	        	d=SmartDashboard.getNumber("kD", RobotMap.driveVelocityLeftPID.getD());
	        	f1=SmartDashboard.getNumber("kFL", RobotMap.driveVelocityLeftPID.getF());
	        	f2=SmartDashboard.getNumber("kFR", RobotMap.driveVelocityRightPID.getF());
	        	IZoneMin=SmartDashboard.getNumber("IZoneMin", RobotMap.driveVelocityLeftPID.getIZoneMin());
	        	IZoneMax=SmartDashboard.getNumber("IZoneMax", RobotMap.driveVelocityLeftPID.getIZoneMax());
	        	
	        	accumL=RobotMap.driveVelocityLeftPID.getErrorSum();
	        	accumR=RobotMap.driveVelocityRightPID.getErrorSum();
	        	outputMin=SmartDashboard.getNumber("outputMin", RobotMap.driveVelocityLeftPID.getMinOut());
	        	outputMax=SmartDashboard.getNumber("outputMax", RobotMap.driveVelocityLeftPID.getMaxOut());
	        }else{
	        	errL=leftOut-(RobotMap.left1.getOutputVoltage()/RobotMap.left1.getBusVoltage());
	        	errR=-rightOut-(RobotMap.right1.getOutputVoltage()/RobotMap.right1.getBusVoltage());
				p=SmartDashboard.getNumber("kP", 0.0);
	        	i=SmartDashboard.getNumber("kI", 0.0);
	        	d=SmartDashboard.getNumber("kD", 0.0);
	        	f1=SmartDashboard.getNumber("kFL", 0.0);
	        	f2=SmartDashboard.getNumber("kFR", 0.0);
	        	IZoneMin=SmartDashboard.getNumber("IZoneMin", 0.0);
	        	IZoneMax=SmartDashboard.getNumber("IZoneMax", Double.MAX_VALUE);
	        	
	        	accumL=RobotMap.drivePositionLeftPID.getErrorSum();
	        	accumR=RobotMap.drivePositionRightPID.getErrorSum();
	        	outputMin=SmartDashboard.getNumber("outputMin", -0.52);
	        	outputMax=SmartDashboard.getNumber("outputMax", 0.52);
	        }
	        accumG=RobotMap.driveGyroPID.getErrorSum();
	        map.put("Time", Timer.getFPGATimestamp()-Robot.closeLoopTime);
	        map.put("LeftError", errL);
	        map.put("RightError", errR);
	        map.put("LeftPosition",  RobotState.getLeftPos());
	        map.put("RightPosition", RobotState.getRightPos());
	        map.put("LeftVelocity", RobotState.getLeftSpeed());
	        map.put("RightVelocity", RobotState.getRightSpeed());
	        map.put("LeftSetpoint", setpointLeft);
	        map.put("RightSetpoint", setpointRight);
	        map.put("kP", p);
	        map.put("kI", i);
	        map.put("kD", d);
	        map.put("kFL", f1);
	        map.put("kFR", f2);
	        map.put("kR", SmartDashboard.getNumber("kR",RobotMap.VelocityRampRate));
			p=SmartDashboard.getNumber("kGP", RobotMap.driveGyroPID.getP());
	    	i=SmartDashboard.getNumber("kGI", RobotMap.driveGyroPID.getI());
	    	d=SmartDashboard.getNumber("kGD", RobotMap.driveGyroPID.getD());
	        map.put("kGP", p);
	        map.put("kGI", i);
	        map.put("kGD", d);
	        map.put("Voltage", DriverStation.getInstance().getBatteryVoltage());
	        map.put("Heading", RobotMap.spig.getAngle());
	        map.put("HeadingSetpoint", setpointHeading);
	        map.put("LeftOutput", leftOut);
	        map.put("RightOutput", -rightOut);
	        map.put("LeftVoltOut", RobotMap.left1.getOutputVoltage()/RobotMap.left1.getBusVoltage());
	        map.put("RightVoltOut", RobotMap.right1.getOutputVoltage()/RobotMap.right1.getBusVoltage());
	    	map.put("outputMin", outputMin);
	    	map.put("outputMax", outputMax);
	    	map.put("ErrorSumLeft", accumL);
	    	map.put("ErrorSumRight", accumR);
	    	map.put("ErrorSumGyro", accumG);
	    	map.put("IZoneMin", IZoneMin);
	    	map.put("IZoneMax", IZoneMax);
	    	map.put("kGIZoneMin", SmartDashboard.getNumber("kGIZoneMin", RobotMap.driveGyroPID.getIZoneMin()));
	    	map.put("kGIZoneMax", SmartDashboard.getNumber("kGIZoneMax", RobotMap.driveGyroPID.getIZoneMax()));
	    	map.put("ShooterSpeed", RobotState.getShooterSpeed());
	    	//map.put("ShootKp", SmartDashboard.getNumber("ShootKp", 0.0));
	    	map.put("ShootErrorSum", RobotMap.shootPID.getErrorSum());
	    	map.put("ShootSetpoint", RobotMap.shootPID.getSetpoint());
	    	map.put("ShootError", RobotMap.shootPID.getError());
	    	//Make sure not to add these for compbot; the processor can only handle so much
	    	map.put("NAVXYaw", (double)RobotMap.navxGyro.getYaw());
	    	map.put("NAVXPitch", (double)RobotMap.navxGyro.getPitch());
	    	map.put("NAVXRoll", (double)RobotMap.navxGyro.getRoll());
	    	map.put("NAVXCompass", (double)RobotMap.navxGyro.getCompassHeading());
	    	map.put("NAVXFullHeading", (double)fusedAngle.getAngle());
	    	map.put("NAVXTotalYaw", (double)RobotMap.navxGyro.getAngle());
	    	map.put("NAVXYawRate", (double)RobotMap.navxGyro.getRate());
	    	map.put("NAVXAccelX", (double)RobotMap.navxGyro.getWorldLinearAccelX());
	    	map.put("NAVXAccelY", (double)RobotMap.navxGyro.getWorldLinearAccelY());
	    	map.put("NAVXAccelZ", (double)RobotMap.navxGyro.getWorldLinearAccelZ());
	    	map.put("NAVXVelocityX", (double)RobotMap.navxGyro.getVelocityX());
	    	map.put("NAVXVelocityY", (double)RobotMap.navxGyro.getVelocityY());
	    	map.put("NAVXVelocityZ", (double)RobotMap.navxGyro.getVelocityZ());
	    	map.put("NAVXDisplacementX", (double)RobotMap.navxGyro.getDisplacementX());
	    	map.put("NAVXDisplacementY", (double)RobotMap.navxGyro.getDisplacementY());
	    	map.put("NAVXDisplacementZ", (double)RobotMap.navxGyro.getDisplacementZ());
	    	map.put("NAVXRawGyroX", (double)RobotMap.navxGyro.getRawGyroX());
	    	map.put("NAVXRawGyroY", (double)RobotMap.navxGyro.getRawGyroY());
	    	map.put("NAVXRawGyroZ", (double)RobotMap.navxGyro.getRawGyroZ());
	    	map.put("NAVXRawAccelX", (double)RobotMap.navxGyro.getRawAccelX());
	    	map.put("NAVXRawAccelY", (double)RobotMap.navxGyro.getRawAccelY());
	    	map.put("NAVXRawAccelZ", (double)RobotMap.navxGyro.getRawAccelZ());
	    	map.put("NAVXRawMagX", (double)RobotMap.navxGyro.getRawMagX());
	    	map.put("NAVXRawMagY", (double)RobotMap.navxGyro.getRawMagY());
	    	map.put("NAVXRawMagZ", (double)RobotMap.navxGyro.getRawMagZ());
	    	map.put("NAVXTemp", (double)RobotMap.navxGyro.getTempC());
	    	map.put("NAVXTimestamp", (double)RobotMap.navxGyro.getLastSensorTimestamp());
	    	map.put("NAVXQuaternionX", (double)RobotMap.navxGyro.getQuaternionX());
	    	map.put("NAVXQuaternionY", (double)RobotMap.navxGyro.getQuaternionY());
	    	map.put("NAVXQuaternionZ", (double)RobotMap.navxGyro.getQuaternionZ());
	    	map.put("NAVXQuaternionW", (double)RobotMap.navxGyro.getQuaternionW());
		}
	}
	public synchronized Map<String, Double> getValueTable(){
		synchronized(map){
			return map;
		}
	}
	public void updatePID(){
		double p,i,d,f1,f2, minOut, maxOut, IZoneMin, IZoneMax;
		if(getCurrentState()==DriveControlState.velocity){
			p=SmartDashboard.getNumber("kP", RobotMap.driveVelocityLeftPID.getP());
        	i=SmartDashboard.getNumber("kI", RobotMap.driveVelocityLeftPID.getI());
        	d=SmartDashboard.getNumber("kD", RobotMap.driveVelocityLeftPID.getD());
        	f1=SmartDashboard.getNumber("kFL", RobotMap.driveVelocityLeftPID.getF());
        	f2=SmartDashboard.getNumber("kFR", RobotMap.driveVelocityRightPID.getF());
        	minOut=SmartDashboard.getNumber("outputMin", RobotMap.driveVelocityLeftPID.getMinOut());
        	maxOut=SmartDashboard.getNumber("outputMax", RobotMap.driveVelocityLeftPID.getMaxOut());
        	IZoneMin=SmartDashboard.getNumber("IZoneMin", RobotMap.driveVelocityLeftPID.getIZoneMin());
        	IZoneMax=SmartDashboard.getNumber("IZoneMax", RobotMap.driveVelocityLeftPID.getIZoneMax());
        	
        	RobotMap.driveVelocityLeftPID.setOutputRange(minOut, maxOut);
        	RobotMap.driveVelocityRightPID.setOutputRange(minOut, maxOut);
        	RobotMap.driveVelocityLeftPID.setPIDF(p, i, d, f1);
        	RobotMap.driveVelocityRightPID.setPIDF(p, i, d, f2);
        	RobotMap.driveVelocityLeftPID.setIZoneRange(IZoneMin,IZoneMax);
        	RobotMap.driveVelocityRightPID.setIZoneRange(IZoneMin,IZoneMax);
		}else{
			p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
        	i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
        	d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
        	f1=SmartDashboard.getNumber("kFL", RobotMap.drivePositionLeftPID.getF());
        	f2=SmartDashboard.getNumber("kFR", RobotMap.drivePositionRightPID.getF());
        	minOut=SmartDashboard.getNumber("outputMin", RobotMap.drivePositionLeftPID.getMinOut());
        	maxOut=SmartDashboard.getNumber("outputMax", RobotMap.drivePositionLeftPID.getMaxOut());
        	IZoneMin=SmartDashboard.getNumber("IZoneMin", RobotMap.drivePositionLeftPID.getIZoneMin());
        	IZoneMax=SmartDashboard.getNumber("IZoneMax", RobotMap.drivePositionLeftPID.getIZoneMax());
        	
        	RobotMap.drivePositionLeftPID.setOutputRange(minOut, maxOut);
        	RobotMap.drivePositionRightPID.setOutputRange(minOut, maxOut);
        	RobotMap.drivePositionLeftPID.setPIDF(p, i, d, f1);
        	RobotMap.drivePositionRightPID.setPIDF(p, i, d, f2);
        	RobotMap.drivePositionLeftPID.setIZoneRange(IZoneMin,IZoneMax);
        	RobotMap.drivePositionRightPID.setIZoneRange(IZoneMin,IZoneMax);
		}
		p=SmartDashboard.getNumber("kGP", RobotMap.driveGyroPID.getP());
    	i=SmartDashboard.getNumber("kGI", RobotMap.driveGyroPID.getI());
    	d=SmartDashboard.getNumber("kGD", RobotMap.driveGyroPID.getD());
    	IZoneMin=SmartDashboard.getNumber("kGIZoneMin", RobotMap.driveGyroPID.getIZoneMin());
    	IZoneMax=SmartDashboard.getNumber("kGIZoneMax", RobotMap.driveGyroPID.getIZoneMax());
    	RobotMap.driveGyroPID.setPID(p, i, d);
    	RobotMap.driveGyroPID.setIZoneRange(IZoneMin,IZoneMax);
    	RobotMap.left1.setVoltageRampRate(SmartDashboard.getNumber("kR",RobotMap.VelocityRampRate));
    	RobotMap.right1.setVoltageRampRate(SmartDashboard.getNumber("kR",RobotMap.VelocityRampRate));
	}
}
