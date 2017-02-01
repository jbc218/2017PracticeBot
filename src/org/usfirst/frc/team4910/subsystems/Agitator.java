package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
/**
 * 
 * @author Jason Cobb
 * Purpose is to move the balls around somehow. I wish it were possible to make the speed proportional to the ball count
 * but we haven't gotten there yet.
 */
public class Agitator {
	private Agitator instance;
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
	private Agitator(){
		
	}
	public Agitator getInstance(){
		return instance==null ? instance=new Agitator() : instance;
	}
}
