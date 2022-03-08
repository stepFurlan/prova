package provaDyn;

import java.util.List;

import org.dyn4j.samples.framework.SimulationBody;
import org.dyn4j.world.World;

public class Touch {
	
	private List<SimulationBody> bodyList;
	
	public Touch(List<SimulationBody> bodyList) {
		this.bodyList = bodyList;
	}
	
	public double[] touch(World world) {
		int counter = 0;
		for(SimulationBody body: this.bodyList) {
			counter++;
			List<SimulationBody> inContactBodies = world.getInContactBodies(body,true);
			for(SimulationBody inContactBody: inContactBodies) {
				Object userData = inContactBody.getUserData();
				if(userData == null) {
					return new double[] {1d};
				}else if(userData != body.getUserData() && userData != null) {
					return new double[] {-1d};
				}else if(userData == body.getUserData()) {
					return new double[] {2d};
				}
			}
		}
		return new double[] {0d};
	}
}
