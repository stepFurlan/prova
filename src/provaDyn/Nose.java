package provaDyn;

import java.util.List;

import org.dyn4j.geometry.Vector2;
import org.dyn4j.samples.framework.SimulationBody;

public class Nose {

	public static final double smellRadius = 7;

	public Nose() {

	}

	// the function returns the number of food items closer than 'smellRadius' value
	// from the worm head
	public int smell(SimulationBody head, List<SimulationBody> foodCoordinates) {
		int foodCounter = 0;
		for (SimulationBody food : foodCoordinates) {
			if (Math.abs(food.getWorldCenter().x - head.getWorldCenter().x) <= smellRadius
					&& Math.abs(food.getWorldCenter().y - head.getWorldCenter().y) <= smellRadius) {
				foodCounter++;
			}
		}
		return foodCounter;
	}

	public int[] smell2(SimulationBody head, List<SimulationBody> foodCoordinates, Vector2 previousBodyCenter) {
		int[] results = new int[2];
		int counterRight = 0;
		int counterLeft = 0;
		Vector2 directionVector = head.getWorldCenter().subtract(previousBodyCenter);
		for (SimulationBody food : foodCoordinates) {

			Vector2 foodVector = head.getWorldCenter().subtract(food.getWorldCenter());
			double product = foodVector.cross(directionVector);
			if (product >= 0 && Math.abs(food.getWorldCenter().x - head.getWorldCenter().x) <= smellRadius
					&& Math.abs(food.getWorldCenter().y - head.getWorldCenter().y) <= smellRadius) {
				counterLeft++;
				results[0] = counterLeft;
			} else if (product < 0 && Math.abs(food.getWorldCenter().x - head.getWorldCenter().x) <= smellRadius
					&& Math.abs(food.getWorldCenter().y - head.getWorldCenter().y) <= smellRadius) {
				counterRight++;
				results[1] = counterRight;
			}

		}
		return results;
	}

}
