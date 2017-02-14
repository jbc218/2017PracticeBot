package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * 
 * @author Jason Cobb
 * Handles both the shooter and the shooting guider. Guider wheel is meant to feed balls into the shooter for some fixed amount of time
 * and at some fixed velocity;
 * 
 */
public class Shooter {

	private static Shooter instance;
	private enum ShooterState{
		Idle, Loading, Shooting;
	}
	private ShooterState currentState=ShooterState.Idle;
	private double loadingStart=0;
	private double shootingStart=0;
	private double shootKp=0.0;
	private double setpoint=SmartDashboard.getNumber("ShootSetpoint", 0.0);
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
			RobotMap.shootControl.set(0);
			RobotMap.shootGuide.set(0);
			currentState=ShooterState.Idle;
			loadingStart=0;
			shootingStart=0;
		}

		@Override
		public void exec() {
			//TODO: fix all this code
			synchronized(Shooter.this){
				ShooterState newState;
				SmartDashboard.putNumber("ShootSetpoint", setpoint);
				shootKp = SmartDashboard.getNumber("ShootKp", 0.0);
				if(OI.rightStick.getRawButton(1) && currentState!=ShooterState.Idle){
					while(OI.rightStick.getRawButton(1)){
						RobotMap.shootControl.set(0);
						RobotMap.shootGuide.set(0);
						newState=ShooterState.Idle;
						currentState=ShooterState.Idle;
					}
				}
				
				switch(currentState){
				case Idle:
					newState=ShooterState.Idle;
					RobotMap.shootControl.set(0);
					RobotMap.shootGuide.set(0);
					
					if(OI.rightStick.getRawButton(OI.ShooterToggle)){
						while(OI.rightStick.getRawButton(OI.ShooterToggle));
						newState = ShooterState.Loading;
						loadingStart=Timer.getFPGATimestamp();
					}
					break;
				case Loading:
					newState = ShooterState.Shooting;
					//Get shooter ready
					while((Timer.getFPGATimestamp()-loadingStart)<RobotMap.shooterSpinupTime && !OI.rightStick.getRawButton(1)){
						RobotMap.shootControl.set((-.60));
						if(OI.rightStick.getRawButton(1)){
							newState=ShooterState.Idle;
							break;
									
						}
					}
					if(newState!=ShooterState.Idle){
						//When shooter is ready, load
						RobotMap.shootGuide.set(1);
						double t = Timer.getFPGATimestamp();
					
						while(Timer.getFPGATimestamp()-t<.5 && !OI.rightStick.getRawButton(1));
						while(OI.rightStick.getRawButton(1))
							newState=ShooterState.Idle;
					}
					break;
				case Shooting:
					//RobotMap.shootGuide.set(0.0);
					//RobotMap.shootPID.calculate(OI.thirdStick.getY());
					//RobotMap.shootControl.set((OI.thirdStick.getY())+(shootKp*(2600.0*OI.thirdStick.getY()-600.0*(RobotMap.shootControl.getEncVelocity()/80.0))));
					//RobotMap.shootControl.set(OI.thirdStick.getY());
					newState=ShooterState.Shooting;
					RobotMap.shootGuide.set(1.0);
					RobotMap.shootControl.set((-.60));
					while(OI.rightStick.getRawButton(OI.ShooterToggle)){
						RobotMap.shootControl.set(0);
						RobotMap.shootGuide.set(0);
						newState=ShooterState.Idle;
						currentState=ShooterState.Idle;
					}
					break;
				default: 
					newState=currentState;
					break;
				}
				if(newState!=currentState){
					System.out.println("Shooter state changed to: "+newState);
					currentState=newState;
					if(newState==ShooterState.Shooting){
						shootingStart=Timer.getFPGATimestamp();
					}
				}
			}
		}

		@Override
		public void end() {
			RobotMap.shootControl.set(0);
			RobotMap.shootGuide.set(0);
			currentState=ShooterState.Idle;
		}
		
	};
	
	public Iterate getLoop(){
		return iter;
	}
	private Shooter(){
		
	}
	public static Shooter getInstance(){
		return instance==null ? instance=new Shooter() : instance;
	}
	private void load(){
		//Get shooter ready
		while((Timer.getFPGATimestamp()-loadingStart)<RobotMap.shooterSpinupTime && !OI.rightStick.getRawButton(1))
			RobotMap.shootControl.set((-.55));
		
		//When shooter is ready, load
		RobotMap.shootGuide.set(.5);
		double t = Timer.getFPGATimestamp();
		while(Timer.getFPGATimestamp()-t<.5 && !OI.rightStick.getRawButton(1));
		while(OI.rightStick.getRawButton(1));
			//RobotMap.shootPID.calculate(OI.thirdStick.getY()); //TODO: Apply vision tracking
			//RobotMap.shootControl.set(shootKp*(400.0*OI.thirdStick.getY()-RobotMap.shootControl.getEncVelocity()));
	}
	
}
