package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
/**
 * 
 * @author Jason Cobb
 * The purpose of this class is to handle the 20 second endgame where we need to attach onto the rope using vision,
 * detect that we have attached by checking either d(Volt) or navX gyro, and then calculating distance up.
 * 
 * There might be a locking mechanism implemented in the future.
 */
public class Climber {
	private Climber instance;
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
	private Climber(){
		
	}
	public Climber getInstance(){
		return instance==null ? instance=new Climber() : instance;
	}
}
