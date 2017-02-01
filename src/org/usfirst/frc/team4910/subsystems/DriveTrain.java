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
	private double leftOut=0.0,rightOut=0.0;
	private static DriveTrain instance;
	private DriveControlState currentState=DriveControlState.regular;
	public boolean hasIterated=false;
	private boolean headingMode=false;
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
//		if(headingMode){
//			G=RobotMap.driveGyroPID.calculate(RobotMap.spig.getAngle());
//			if(RobotMap.driveGyroPID.onTarget())
//				disableHeadingMode();
//		}
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
//		double G=0.0;
//		if(headingMode){
//			G=RobotMap.driveGyroPID.calculate(RobotMap.spig.getAngle());
//			
//		}
		RobotMap.drivePositionLeftPID.setOnTime(Timer.getFPGATimestamp()-currentStateStartTime);
		RobotMap.drivePositionRightPID.setOnTime(Timer.getFPGATimestamp()-currentStateStartTime);
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
		//RobotMap.drivePositionLeftPID.setOnTime(Timer.getFPGATimestamp()-currentStateStartTime);
		//RobotMap.drivePositionRightPID.setOnTime(Timer.getFPGATimestamp()-currentStateStartTime);
//		if(headingMode){
//			double G=RobotMap.driveGyroPID.calculate(RobotMap.spig.getAngle());
//			RobotMap.driveVelocityLeftPID.setSetpoint(RobotMap.leftMaxRPM*(setpointLeft-G));
//			RobotMap.driveVelocityRightPID.setSetpoint(RobotMap.rightMaxRPM*(setpointRight-G));
//		}
		drive(RobotMap.driveVelocityLeftPID.calculate(rpmToInchesPerSecond(RobotMap.left1.getSpeed())),
				RobotMap.driveVelocityRightPID.calculate(rpmToInchesPerSecond(RobotMap.right1.getSpeed()))); //was negative earlier for some reason
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
//	public synchronized void setHeadingSetpoint(double set){
//		setpointHeading=set;
//		RobotMap.driveGyroPID.setSetpoint(set);
//		headingMode=true;
//	}
//	public synchronized void disableHeadingMode(){
//		headingMode=false;
//		RobotMap.spig.reset();
//		resetAll();
//	}
	public synchronized double[] getSetpoints(){
		return new double[]{setpointLeft,setpointRight};
	}
	private synchronized void drive(double left, double right){
		//System.out.println(left);
		//double G = headingMode ? RobotMap.driveGyroPID.calculate(RobotMap.spig.getAngle()) : 0;
		leftOut=left;
		rightOut=-right;
        if ( Math.abs(left) < .15 && !headingMode && currentState==DriveControlState.regular) {
			//RobotMap.driveVelocityLeftPID.setSetpoint(3200*(0-G));
			//RobotMap.left1.set(RobotMap.driveVelocityLeftPID.calculate(-RobotMap.left1.getEncVelocity()));
        	RobotMap.left1.set(0); //0-G
        }else{
			//RobotMap.driveVelocityLeftPID.setSetpoint(3200*(left-G));
			//RobotMap.left1.set(RobotMap.driveVelocityLeftPID.calculate(-RobotMap.left1.getEncVelocity()));
		
        	RobotMap.left1.set(left); //left+G
        }
        if ( Math.abs(right) < .15 && !headingMode && currentState==DriveControlState.regular) {
        	RobotMap.right1.set(0); //0-G
			//RobotMap.driveVelocityRightPID.setSetpoint(3200*(0-G));
			//RobotMap.right1.set(RobotMap.driveVelocityRightPID.calculate(RobotMap.right1.getEncVelocity()));
        }else{
			//RobotMap.driveVelocityRightPID.setSetpoint(3200*(right-G));
			//RobotMap.right1.set(RobotMap.driveVelocityRightPID.calculate(RobotMap.right1.getEncVelocity()));
        	RobotMap.right1.set(-right); //right-G
        }
	}
	private synchronized void resetAll(){
//		RobotMap.g.reset();
		RobotMap.left1.setPosition(0);
		RobotMap.right1.setPosition(0);
		RobotMap.drivePositionLeftPID.reset();
		RobotMap.drivePositionRightPID.reset();
		RobotMap.drivePositionLeftPID.setSetpoint(setpointLeft);
		RobotMap.drivePositionRightPID.setSetpoint(setpointRight);
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
//        SmartDashboard.putNumber("gyro angle", RobotMap.g.getAngle());
//        SmartDashboard.putNumber("gyro center", RobotMap.g.getCenter());
//        SmartDashboard.putNumber("SPI gyro center", RobotMap.spig.getCenter());
//        SmartDashboard.putNumber("heading error", setpointHeading-RobotMap.spig.getAngle());
        SmartDashboard.putNumber("battery voltage", DriverStation.getInstance().getBatteryVoltage());
           
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
     * @return list of all v in <k, v> with the delimiter "#"
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
	        double errL=0,errR=0, p=0,i=0,d=0,f1=0,f2=0,v=0,outputMin=-1.0, outputMax=1.0, accumL=0.0, accumR=0.0, accumG=0.0, IZone=0.0;
	        if(currentState==DriveControlState.position){
	        	errL=RobotMap.drivePositionLeftPID.getError();
	        	errR=RobotMap.drivePositionRightPID.getError();
				p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
	        	i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
	        	d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
	        	f1=SmartDashboard.getNumber("kFL", RobotMap.drivePositionLeftPID.getF());
	        	f2=SmartDashboard.getNumber("kFR", RobotMap.drivePositionRightPID.getF());
	        	v=SmartDashboard.getNumber("kV", RobotMap.drivePositionLeftPID.getV());
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
	        	v=SmartDashboard.getNumber("kV", RobotMap.driveVelocityLeftPID.getV());
	        	IZone=SmartDashboard.getNumber("IZone", RobotMap.driveVelocityLeftPID.getIZone());
	        	accumL=RobotMap.driveVelocityLeftPID.getErrorSum();
	        	accumR=RobotMap.driveVelocityRightPID.getErrorSum();
	        	outputMin=SmartDashboard.getNumber("outputMin", RobotMap.driveVelocityLeftPID.getMinOut());
	        	outputMax=SmartDashboard.getNumber("outputMax", RobotMap.driveVelocityLeftPID.getMaxOut());
	        }else{
	        	errL=leftOut-(RobotMap.left1.getOutputVoltage()/RobotMap.left1.getBusVoltage());
	        	errR=-rightOut-(RobotMap.right1.getOutputVoltage()/RobotMap.right1.getBusVoltage());
				p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
	        	i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
	        	d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
	        	f1=SmartDashboard.getNumber("kFL", RobotMap.drivePositionLeftPID.getF());
	        	f2=SmartDashboard.getNumber("kFR", RobotMap.drivePositionRightPID.getF());
	        	v=SmartDashboard.getNumber("kV", RobotMap.drivePositionLeftPID.getV());
	        	IZone=SmartDashboard.getNumber("IZone", RobotMap.drivePositionLeftPID.getIZone());
	        	accumL=RobotMap.drivePositionLeftPID.getErrorSum();
	        	accumR=RobotMap.drivePositionRightPID.getErrorSum();
	        	outputMin=SmartDashboard.getNumber("outputMin", RobotMap.drivePositionLeftPID.getMinOut());
	        	outputMax=SmartDashboard.getNumber("outputMax", RobotMap.drivePositionLeftPID.getMaxOut());
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
	        map.put("kV", v);
	        map.put("kGP", RobotMap.driveGyroPID.getP());
	        map.put("kGI", RobotMap.driveGyroPID.getI());
	        map.put("kGD", RobotMap.driveGyroPID.getD());
	        map.put("Voltage", DriverStation.getInstance().getBatteryVoltage());
//	        map.put("Heading", RobotMap.spig.getAngle());
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
	    	map.put("kGIZone", SmartDashboard.getNumber("kGIzone", RobotMap.driveGyroPID.getIZone()));
		}
	}
	public synchronized Map<String, Double> getValueTable(){
		synchronized(map){
			return map;
		}
	}
	public void updatePID(){
		double p,i,d,f1,f2,v, minOut, maxOut, IZone;
		if(getCurrentState()==DriveControlState.velocity){
			p=SmartDashboard.getNumber("kP", RobotMap.driveVelocityLeftPID.getP());
        	i=SmartDashboard.getNumber("kI", RobotMap.driveVelocityLeftPID.getI());
        	d=SmartDashboard.getNumber("kD", RobotMap.driveVelocityLeftPID.getD());
        	f1=SmartDashboard.getNumber("kFL", RobotMap.driveVelocityLeftPID.getF());
        	f2=SmartDashboard.getNumber("kFR", RobotMap.driveVelocityRightPID.getF());
        	v=SmartDashboard.getNumber("kV", RobotMap.driveVelocityLeftPID.getV());
        	minOut=SmartDashboard.getNumber("outputMin", RobotMap.driveVelocityLeftPID.getMinOut());
        	maxOut=SmartDashboard.getNumber("outputMax", RobotMap.driveVelocityLeftPID.getMaxOut());
        	IZone=SmartDashboard.getNumber("IZone", RobotMap.driveVelocityLeftPID.getIZone());
        	RobotMap.driveVelocityLeftPID.setOutputRange(minOut, maxOut);
        	RobotMap.driveVelocityRightPID.setOutputRange(minOut, maxOut);
        	RobotMap.driveVelocityLeftPID.setPIDFV(p, i, d, f1, v);
        	RobotMap.driveVelocityRightPID.setPIDFV(p, i, d, f2, v);
        	RobotMap.driveVelocityLeftPID.setIZone(IZone);
        	RobotMap.driveVelocityRightPID.setIZone(IZone);
		}else{
			p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
        	i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
        	d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
        	f1=SmartDashboard.getNumber("kFL", RobotMap.drivePositionLeftPID.getF());
        	f2=SmartDashboard.getNumber("kFR", RobotMap.drivePositionRightPID.getF());
        	v=SmartDashboard.getNumber("kV", RobotMap.drivePositionLeftPID.getV());
        	minOut=SmartDashboard.getNumber("outputMin", RobotMap.drivePositionLeftPID.getMinOut());
        	maxOut=SmartDashboard.getNumber("outputMax", RobotMap.drivePositionLeftPID.getMaxOut());
        	IZone=SmartDashboard.getNumber("IZone", RobotMap.drivePositionLeftPID.getIZone());
        	
        	RobotMap.drivePositionLeftPID.setOutputRange(minOut, maxOut);
        	RobotMap.drivePositionRightPID.setOutputRange(minOut, maxOut);
        	RobotMap.drivePositionLeftPID.setPIDFV(p, i, d, f1, v);
        	RobotMap.drivePositionRightPID.setPIDFV(p, i, d, f2, v);
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
