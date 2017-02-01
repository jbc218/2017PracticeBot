package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;


/**
 * 
 * @author Jason Cobb
 * Handles both the shooter and the shooting guider. Guider wheel is meant to feed balls into the shooter for some fixed amount of time
 * and at some fixed velocity;
 * 
 */
public class Shooter {

	private Shooter instance;
	private enum ShooterState{
		Idle, Loading, Shooting;
	}
	private ShooterState currentState=ShooterState.Idle;
	private double loadingStart=0;
	private double shootingStart=0;
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
			RobotMap.shootControl.set(0);
			RobotMap.shootGuide.set(0);
			currentState=ShooterState.Idle;
		}

		@Override
		public void exec() {
			//TODO: look over this code again
			synchronized(Shooter.this){
				ShooterState newState;
				switch(currentState){
				case Idle:
					newState=ShooterState.Idle;
					if(OI.leftStick.getRawButton(1)){
						newState = ShooterState.Loading;
						loadingStart=Timer.getFPGATimestamp();
					}
					break;
				case Loading:
					load();
					newState = (Timer.getFPGATimestamp()-loadingStart)>RobotMap.shooterGuideOptimalTime ? ShooterState.Shooting : ShooterState.Shooting;
					break;
				case Shooting:
					RobotMap.shootPID.calculate(OI.thirdStick.getY());
					newState = (Timer.getFPGATimestamp()-shootingStart)>RobotMap.shooterTimeToShoot ? ShooterState.Idle : ShooterState.Shooting;
					break;
				default: 
					newState=currentState;
					break;
				}
				if(newState!=currentState){
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
	public Shooter getInstance(){
		return instance==null ? instance=new Shooter() : instance;
	}
	private void load(){
		RobotMap.shootGuide.set(RobotMap.shootGuidePID.calculate(RobotMap.shooterGuideOptimalSpeed));
		if((Timer.getFPGATimestamp()-loadingStart)>RobotMap.shooterSpinupTime)
			RobotMap.shootPID.calculate(OI.thirdStick.getY()); //TODO: Apply vision tracking
	}
	
}
