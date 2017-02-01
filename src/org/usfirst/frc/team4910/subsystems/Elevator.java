package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;

/**
 * 
 * @author Jason Cobb
 * Handles the ball intake mechanism. Motors go at a speed proportional to the driving motors, and activate and deactivate 
 * at the choice of the operator.
 */
public class Elevator {
	private Elevator instance;
	private enum ElevatorState{
		Disabled, Running;
	}
	private ElevatorState currentState=ElevatorState.Running;
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
			RobotMap.elevControl.set(0);
			currentState=ElevatorState.Disabled;
		}

		@Override
		public void exec() {
			synchronized(Elevator.this){
				switch(currentState){
				case Disabled:
					if(OI.thirdStick.getRawButton(2)){
						while(OI.thirdStick.getRawButton(2)){
							RunElevator();
						}
						currentState=ElevatorState.Running;
					}
					currentState=ElevatorState.Disabled;
					break;
				case Running:
					if(OI.thirdStick.getRawButton(2)){
						while(OI.thirdStick.getRawButton(2)){
							RobotMap.elevControl.set(0);
						}
						currentState=ElevatorState.Disabled;
					}
					currentState=ElevatorState.Running;
					break;
				default:
					currentState=ElevatorState.Disabled;
					break;
				}
			}
		}

		@Override
		public void end() {
			RobotMap.elevControl.set(0);
			currentState=ElevatorState.Disabled;
		}
		
	};
	
	public Iterate getLoop(){
		return iter;
	}
	private Elevator(){
		
	}
	public Elevator getInstance(){
		return instance==null ? instance=new Elevator() : instance;
	}
	private synchronized void RunElevator(){
		// o = (b+f) - x
		// o is output, b is the maximum possible motor average, f is the minimum elevator speed
		// x is the average of the two drive motor outputs
		RobotMap.elevControl.set(1.25-((RobotMap.left1.get()+RobotMap.right1.get())/2.0));
		
	}
}
