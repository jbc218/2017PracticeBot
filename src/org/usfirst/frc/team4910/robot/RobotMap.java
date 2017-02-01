package org.usfirst.frc.team4910.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Calendar;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team4910.util.*;
import com.ctre.*;
import com.kauailabs.navx.frc.AHRS;
import com.opencsv.CSVWriter;
public class RobotMap {
	
	public static CANTalon left1,left2,right1,right2;
	public static CANTalon elevControl, shootControl, climbControl, agitateControl, shootGuide;
	public static DoubleSolenoid gearShifter;
	public static Compressor c;
	public static SynchronousPID drivePositionLeftPID,drivePositionRightPID,driveVelocityLeftPID,driveVelocityRightPID;
	public static SynchronousPID driveGyroPID;
	public static SynchronousPID shootPID, shootGuidePID; //both should be a simple PD+F loop
	public static AHRS navxGyro;
	public static GyroHelper spig;
	public static CSVWriter writer;
	
	public static final double DriveWheelDiameter=6;
	public static final double EncCountsPerRev=4096;
	public static final double rightMinRPM=-480;
	public static final double rightMaxRPM=480;
	public static final double leftMinRPM=-480;
	public static final double leftMaxRPM=480;
	
	public static final double PositionKp=0.0;
	public static final double PositionKi=0.0;
	public static final double PositionKd=0.0;
	public static final double PositionKf=0.0;
	public static final double VelocityKp=0.0;
	public static final double VelocityKi=0.0;
	public static final double VelocityKd=0.0;
	public static final double VelocityKf1=0.0;
	public static final double VelocityKf2=0.0;
	public static final double GyroKp=0.0;
	public static final double GyroKi=0.0;
	public static final double GyroKd=0.0;
	public static final double shooterKp=0.0;
	public static final double shooterKd=0.0;
	public static final double shooterKf=0.0;
	public static final double shooterGuideKp=0.0;
	public static final double shooterGuideKd=0.0;
	public static final double shooterGuideKf=0.0;
	public static final double shooterGuideOptimalSpeed=0.0;
	public static final double shooterGuideOptimalTime=0.0;
	public static final double shooterSpinupTime=0.0;
	public static final double shooterTimeToShoot=0.0;
	public static double VelocityRampRate=24.0;
	
	public static void init(){
//		try{
//			Calendar now = Calendar.getInstance();
//			String str = String.valueOf(now.get(Calendar.MONTH))+"."+String.valueOf(now.get(Calendar.DAY_OF_MONTH))+"."
//					+String.valueOf(now.get(Calendar.HOUR_OF_DAY))+"."+String.valueOf(now.get(Calendar.MINUTE))+"."+String.valueOf(now.get(Calendar.SECOND));
//			
//			writer = new CSVWriter(new FileWriter("TestingData"+str+".csv"), ',');
//			String[] tabNames = ("Time#Left Error#Right Error#Left Position#Right Position#Left Velocity#Right Velocity#Left Setpoint"
//					+ "#Right Setpoint#Weighted Left Error#Weighted Right Error#Weighted Left Position#Weighted Right Position"
//					+ "#Weighted Left Velocity#Weighted Right Velocity#Weighted Left Setpoint#Weighted Right Setpoint#kP#kI#kD").split("#");
//			writer.writeNext(tabNames, true);
//		}catch(IOException e){
//			e.printStackTrace();
//		}
		left1 = new CANTalon(4);
		left2 = new CANTalon(3);
		right1 = new CANTalon(2);
		right2 = new CANTalon(1);
		spig = new GyroHelper(SPI.Port.kOnboardCS0);
		navxGyro = new AHRS(SPI.Port.kMXP);
		left1.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		right1.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        left1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        left1.set(0);
        left2.changeControlMode(CANTalon.TalonControlMode.Follower);
        left2.set(left1.getDeviceID());
        right1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        right1.set(0);
        right2.changeControlMode(CANTalon.TalonControlMode.Follower);
        right2.set(right1.getDeviceID());
        gearShifter = new DoubleSolenoid(0,1);
        c = new Compressor(0); //TODO: Schedule the compressor for certain times during the match
        c.start();
        drivePositionLeftPID = new SynchronousPID(PositionKp,PositionKi,PositionKd,PositionKf);
        drivePositionRightPID = new SynchronousPID(PositionKp,PositionKi,PositionKd,PositionKf);
        driveVelocityLeftPID = new SynchronousPID(VelocityKp,VelocityKi,VelocityKd,VelocityKf1);
        driveVelocityRightPID = new SynchronousPID(VelocityKp,VelocityKi,VelocityKd,VelocityKf2);
        driveVelocityLeftPID.setOutputRange(-3200, 3200);
        driveVelocityRightPID.setOutputRange(-3200, 3200);
        driveGyroPID = new SynchronousPID(GyroKp, GyroKi, GyroKd);
        shootPID = new SynchronousPID(shooterKp, 0.0, shooterKd, shooterKf);
        shootGuidePID = new SynchronousPID(shooterGuideKp, 0.0, shooterGuideKd, shooterGuideKf);
        left1.configEncoderCodesPerRev((int)(EncCountsPerRev));
        right1.configEncoderCodesPerRev((int)(EncCountsPerRev));
        left1.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        right1.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        
        left1.reverseSensor(true);
        left1.reverseOutput(true);
        left2.reverseOutput(false);
        
        right1.reverseSensor(false);
        right1.reverseOutput(true);
        right2.reverseOutput(false);
		
	}
}
