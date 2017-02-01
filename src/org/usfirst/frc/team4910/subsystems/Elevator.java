package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;

/**
 * 
 * @author Jason Cobb
 * Handles the ball intake mechanism. Motors go at a speed proportional to the driving motors, and activate and deactivate 
 * at the choice of the operator.
 */
public class Elevator {
	private Elevator instance;
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
		}

		@Override
		public void exec() {
		}

		@Override
		public void end() {
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
}
