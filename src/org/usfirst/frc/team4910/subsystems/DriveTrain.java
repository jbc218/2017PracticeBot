package org.usfirst.frc.team4910.subsystems;

import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveControlState;
import org.usfirst.frc.team4910.util.SynchronousPID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	private final Iterate iter = new Iterate(){
        private boolean stateChanged;
        
		@Override
		public void init() {
			updatePID();
			setControlState(DriveControlState.regular);
			resetAll();
			hasIterated=false;
		}

		@Override
		public void exec() {
			synchronized(DriveTrain.this){
				DriveControlState newState;
				double now = Timer.getFPGATimestamp();
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
				outputToDashboard();
				hasIterated=true;
			}
		}

		@Override
		public void end() {
			resetAll();
		}
		
	};
	
	private DriveTrain(){
		updatePID();
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
		drive(RobotMap.drivePositionLeftPID.calculate(countsToInches(-RobotMap.left1.getEncPosition())), 
				RobotMap.drivePositionRightPID.calculate(countsToInches(RobotMap.right1.getEncPosition())));
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
			RobotMap.driveVelocityLeftPID.setSetpoint(RobotMap.leftMaxIPS*(setpointLeft-G));
			RobotMap.driveVelocityRightPID.setSetpoint(RobotMap.rightMaxIPS*(setpointRight-G));
		}
		drive(RobotMap.driveVelocityLeftPID.calculate(rpmToInchesPerSecond(RobotMap.left1.getSpeed())),
				RobotMap.driveVelocityRightPID.calculate(rpmToInchesPerSecond(RobotMap.right1.getSpeed())));
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
		//TODO: implement this sometime later
		//Never mind. Auto doesn't require this much sophistication
		
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
		leftOut=left;
		rightOut=-right;
        if ( Math.abs(left) < .15 && !headingMode && currentState==DriveControlState.regular) {
        	RobotMap.left1.set(0);
        }else{
        	RobotMap.left1.set(left);
        }
        if ( Math.abs(right) < .15 && !headingMode && currentState==DriveControlState.regular) {
        	RobotMap.right1.set(0);
        }else{
        	RobotMap.right1.set(-right);
        }
	}
	public synchronized void resetAll(){
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
	
	public synchronized void outputToDashboard(){
		updateHashTable();
		synchronized(map){
			for(Map.Entry<String, Double> me : map.entrySet()){
				SmartDashboard.putNumber(me.getKey(), me.getValue());
			}
		}
        SmartDashboard.putNumber("SPI gyro center", RobotMap.spig.getCenter());
        SmartDashboard.putNumber("heading error", setpointHeading-RobotMap.spig.getAngle());
        SmartDashboard.putNumber("battery voltage", DriverStation.getInstance().getBatteryVoltage());
        
//        boolean collisionDetected = false;
//        
//        double curr_world_linear_accel_x = RobotMap.navxGyro.getWorldLinearAccelX();
//        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
//        last_world_linear_accel_x = curr_world_linear_accel_x;
//        double curr_world_linear_accel_y = RobotMap.navxGyro.getWorldLinearAccelY();
//        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
//        last_world_linear_accel_y = curr_world_linear_accel_y;
//        
//        if ( ( Math.abs(currentJerkX) > .7f ) ||
//             ( Math.abs(currentJerkY) > .7f) ) { //.5f is the collision threshold
//            collisionDetected = true;
//        }
//        
//        SmartDashboard.putBoolean("Has collided", collisionDetected);
//        SmartDashboard.putBoolean("NAVX Connected", RobotMap.navxGyro.isConnected());
//        SmartDashboard.putBoolean("NAVX Calibrating", RobotMap.navxGyro.isCalibrating());
//        SmartDashboard.putBoolean("NAVX Moving Detected", RobotMap.navxGyro.isMoving());
//        SmartDashboard.putBoolean("NAVX Rotation detected", RobotMap.navxGyro.isRotating());
//        SmartDashboard.putString("NAVX Firmware Version", RobotMap.navxGyro.getFirmwareVersion());
//        SmartDashboard.putNumber("NAVX Pressure", RobotMap.navxGyro.getPressure());
//        SmartDashboard.putNumber("NAVX Barometric Pressure", RobotMap.navxGyro.getBarometricPressure());
           
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
	        double errL=0,errR=0, p=0,i=0,d=0,f1=0,f2=0,outputMin=-1.0, outputMax=1.0, accumL=0.0, accumR=0.0, accumG=0.0, IZone=0.0;
	        if(currentState==DriveControlState.position){
	        	errL=RobotMap.drivePositionLeftPID.getError();
	        	errR=RobotMap.drivePositionRightPID.getError();
				p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
	        	i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
	        	d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
	        	f1=SmartDashboard.getNumber("kFL", RobotMap.drivePositionLeftPID.getF());
	        	f2=SmartDashboard.getNumber("kFR", RobotMap.drivePositionRightPID.getF());
	        	IZone=SmartDashboard.getNumber("IZone", RobotMap.drivePositionLeftPID.getIZone());
	        	
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
	        	IZone=SmartDashboard.getNumber("IZone", RobotMap.driveVelocityLeftPID.getIZone());
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
	        	IZone=SmartDashboard.getNumber("IZone", 0.0);
	        	accumL=RobotMap.drivePositionLeftPID.getErrorSum();
	        	accumR=RobotMap.drivePositionRightPID.getErrorSum();
	        	outputMin=SmartDashboard.getNumber("outputMin", -0.52);
	        	outputMax=SmartDashboard.getNumber("outputMax", 0.52);
	        }
	        accumG=RobotMap.driveGyroPID.getErrorSum();
	        map.put("Time", Timer.getFPGATimestamp()-Robot.closeLoopTime);
	        map.put("LeftError", errL);
	        map.put("RightError", errR);
	        map.put("LeftPosition",  countsToInches(-RobotMap.left1.getEncPosition()));
	        map.put("RightPosition", countsToInches(RobotMap.right1.getEncPosition()));
	        map.put("LeftVelocity", rpmToInchesPerSecond(RobotMap.left1.getSpeed()));
	        map.put("RightVelocity", rpmToInchesPerSecond(RobotMap.right1.getSpeed()));
	        map.put("LeftSetpoint", setpointLeft);
	        map.put("RightSetpoint", setpointRight);
	        map.put("WeightedLeftError", countsToInches(errL));
	        map.put("WeightedRightError", countsToInches(errR));
	        map.put("WeightedLeftPosition", RobotMap.left1.getPosition());
	        map.put("WeightedRightPosition", RobotMap.right1.getPosition());
	        map.put("WeightedLeftVelocity", countsToInches(RobotMap.left1.getEncVelocity()));
	        map.put("WeightedRightVelocity", countsToInches(RobotMap.right1.getEncVelocity()));
	        map.put("WeightedLeftSetpoint", countsToInches(setpointLeft));
	        map.put("WeightedRightSetpoint", countsToInches(setpointRight));
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
	    	map.put("IZone", IZone);
	    	map.put("kGIZone", SmartDashboard.getNumber("kGIZone", 8.0));
//	    	map.put("NAVXYaw", (double)RobotMap.navxGyro.getYaw());
//	    	map.put("NAVXPitch", (double)RobotMap.navxGyro.getPitch());
//	    	map.put("NAVXRoll", (double)RobotMap.navxGyro.getRoll());
//	    	map.put("NAVXCompass", (double)RobotMap.navxGyro.getCompassHeading());
//	    	map.put("NAVXFullHeading", (double)RobotMap.navxGyro.getFusedHeading());
//	    	map.put("NAVXTotalYaw", (double)RobotMap.navxGyro.getAngle());
//	    	map.put("NAVXYawRate", (double)RobotMap.navxGyro.getRate());
//	    	map.put("NAVXAccelX", (double)RobotMap.navxGyro.getWorldLinearAccelX());
//	    	map.put("NAVXAccelY", (double)RobotMap.navxGyro.getWorldLinearAccelY());
//	    	map.put("NAVXAccelZ", (double)RobotMap.navxGyro.getWorldLinearAccelZ());
//	    	map.put("NAVXVelocityX", (double)RobotMap.navxGyro.getVelocityX());
//	    	map.put("NAVXVelocityY", (double)RobotMap.navxGyro.getVelocityY());
//	    	map.put("NAVXVelocityZ", (double)RobotMap.navxGyro.getVelocityZ());
//	    	map.put("NAVXDisplacementX", (double)RobotMap.navxGyro.getDisplacementX());
//	    	map.put("NAVXDisplacementY", (double)RobotMap.navxGyro.getDisplacementY());
//	    	map.put("NAVXDisplacementZ", (double)RobotMap.navxGyro.getDisplacementZ());
//	    	map.put("NAVXRawGyroX", (double)RobotMap.navxGyro.getRawGyroX());
//	    	map.put("NAVXRawGyroY", (double)RobotMap.navxGyro.getRawGyroY());
//	    	map.put("NAVXRawGyroZ", (double)RobotMap.navxGyro.getRawGyroZ());
//	    	map.put("NAVXRawAccelX", (double)RobotMap.navxGyro.getRawAccelX());
//	    	map.put("NAVXRawAccelY", (double)RobotMap.navxGyro.getRawAccelY());
//	    	map.put("NAVXRawAccelZ", (double)RobotMap.navxGyro.getRawAccelZ());
//	    	map.put("NAVXRawMagX", (double)RobotMap.navxGyro.getRawMagX());
//	    	map.put("NAVXRawMagY", (double)RobotMap.navxGyro.getRawMagY());
//	    	map.put("NAVXRawMagZ", (double)RobotMap.navxGyro.getRawMagZ());
//	    	map.put("NAVXTemp", (double)RobotMap.navxGyro.getTempC());
//	    	map.put("NAVXTimestamp", (double)RobotMap.navxGyro.getLastSensorTimestamp());
//	    	map.put("NAVXQuaternionX", (double)RobotMap.navxGyro.getQuaternionX());
//	    	map.put("NAVXQuaternionY", (double)RobotMap.navxGyro.getQuaternionY());
//	    	map.put("NAVXQuaternionZ", (double)RobotMap.navxGyro.getQuaternionZ());
//	    	map.put("NAVXQuaternionW", (double)RobotMap.navxGyro.getQuaternionW());
		}
	}
	public synchronized Map<String, Double> getValueTable(){
		synchronized(map){
			return map;
		}
	}
	public void updatePID(){
		double p,i,d,f1,f2, minOut, maxOut, IZone;
		if(getCurrentState()==DriveControlState.velocity){
			p=SmartDashboard.getNumber("kP", RobotMap.driveVelocityLeftPID.getP());
        	i=SmartDashboard.getNumber("kI", RobotMap.driveVelocityLeftPID.getI());
        	d=SmartDashboard.getNumber("kD", RobotMap.driveVelocityLeftPID.getD());
        	f1=SmartDashboard.getNumber("kFL", RobotMap.driveVelocityLeftPID.getF());
        	f2=SmartDashboard.getNumber("kFR", RobotMap.driveVelocityRightPID.getF());
        	minOut=SmartDashboard.getNumber("outputMin", RobotMap.driveVelocityLeftPID.getMinOut());
        	maxOut=SmartDashboard.getNumber("outputMax", RobotMap.driveVelocityLeftPID.getMaxOut());
        	IZone=SmartDashboard.getNumber("IZone", RobotMap.driveVelocityLeftPID.getIZone());
        	RobotMap.driveVelocityLeftPID.setOutputRange(minOut, maxOut);
        	RobotMap.driveVelocityRightPID.setOutputRange(minOut, maxOut);
        	RobotMap.driveVelocityLeftPID.setPIDF(p, i, d, f1);
        	RobotMap.driveVelocityRightPID.setPIDF(p, i, d, f2);
        	RobotMap.driveVelocityLeftPID.setIZone(IZone);
        	RobotMap.driveVelocityRightPID.setIZone(IZone);
		}else{
			p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
        	i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
        	d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
        	f1=SmartDashboard.getNumber("kFL", RobotMap.drivePositionLeftPID.getF());
        	f2=SmartDashboard.getNumber("kFR", RobotMap.drivePositionRightPID.getF());
        	minOut=SmartDashboard.getNumber("outputMin", RobotMap.drivePositionLeftPID.getMinOut());
        	maxOut=SmartDashboard.getNumber("outputMax", RobotMap.drivePositionLeftPID.getMaxOut());
        	IZone=SmartDashboard.getNumber("IZone", RobotMap.drivePositionLeftPID.getIZone());
        	
        	RobotMap.drivePositionLeftPID.setOutputRange(minOut, maxOut);
        	RobotMap.drivePositionRightPID.setOutputRange(minOut, maxOut);
        	RobotMap.drivePositionLeftPID.setPIDF(p, i, d, f1);
        	RobotMap.drivePositionRightPID.setPIDF(p, i, d, f2);
        	RobotMap.drivePositionLeftPID.setIZone(IZone);
        	RobotMap.drivePositionRightPID.setIZone(IZone);
		}
		p=SmartDashboard.getNumber("kGP", RobotMap.driveGyroPID.getP());
    	i=SmartDashboard.getNumber("kGI", RobotMap.driveGyroPID.getI());
    	d=SmartDashboard.getNumber("kGD", RobotMap.driveGyroPID.getD());
    	IZone=SmartDashboard.getNumber("kGIZone", RobotMap.driveGyroPID.getIZone());
    	RobotMap.driveGyroPID.setPID(p, i, d);
    	RobotMap.driveGyroPID.setIZone(IZone);
    	RobotMap.left1.setVoltageRampRate(SmartDashboard.getNumber("kR",RobotMap.VelocityRampRate));
    	RobotMap.right1.setVoltageRampRate(SmartDashboard.getNumber("kR",RobotMap.VelocityRampRate));
	}
}
