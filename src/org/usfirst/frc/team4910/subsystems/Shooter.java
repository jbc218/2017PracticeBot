package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;


/**
 * 
 * @author Jason Cobb
 * Handles both the shooter and the shooting guider. Apparently the guiding wheel is meant to feed balls into the shooter.
 * I have no real idea how this would work and we haven't finished prototyping it. The prototyping team is almost exclucively
 * first-years so I don't have enough info to proceed past basic button -> spin wheel code.
 */
public class Shooter {

	private Shooter instance;
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
	private Shooter(){
		
	}
	public Shooter getInstance(){
		return instance==null ? instance=new Shooter() : instance;
	}
	
}
