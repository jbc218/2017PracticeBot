package org.usfirst.frc.team4910.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Calendar;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team4910.subsystems.DriveTrain;
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
	public static final double rightMinIPS=DriveTrain.rpmToInchesPerSecond(-480);
	public static final double rightMaxIPS=DriveTrain.rpmToInchesPerSecond(480);
	public static final double leftMinIPS=DriveTrain.rpmToInchesPerSecond(-480);
	public static final double leftMaxIPS=DriveTrain.rpmToInchesPerSecond(480);
	
	//Position, velocity, and gyro PIDF values are all for low gear only
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
        driveVelocityLeftPID.setOutputRange(leftMinIPS, leftMaxIPS);
        driveVelocityRightPID.setOutputRange(rightMinIPS, rightMaxIPS);
        driveGyroPID = new SynchronousPID(GyroKp, GyroKi, GyroKd);
        shootPID = new SynchronousPID(shooterKp, 0.0, shooterKd, shooterKf); //If we have to use I, we're doing something wrong
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
