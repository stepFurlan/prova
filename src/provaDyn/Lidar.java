package provaDyn;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import org.dyn4j.collision.Filter;
import org.dyn4j.geometry.Ray;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.samples.framework.SimulationBody;
import org.dyn4j.samples.framework.SimulationFrame;
import org.dyn4j.world.DetectFilter;
import org.dyn4j.world.World;
import org.dyn4j.world.result.RaycastResult;
public class Lidar {
	
	public static final double rayLength = 50.0;

	private final double[] rayDirections;

	public Lidar() {
		this.rayDirections = new double[9];
		
		this.rayDirections[0] = Math.PI * 1d / 4d;
		this.rayDirections[1] = Math.PI * 7d / 4d;
		this.rayDirections[2] = Math.PI * 1d / 3d;
		this.rayDirections[3] = Math.PI * 5d / 3d;
		this.rayDirections[4] = Math.PI * 1d / 6d;
		this.rayDirections[5] = Math.PI * 11d / 6d;
		this.rayDirections[6] = Math.PI * 2d;
		this.rayDirections[7] = Math.PI * 1d / 12d;
		this.rayDirections[8] = Math.PI * 23d / 12d;
	}

	public double[] view(SimulationBody head, World world, List<Ray> rayList, Vector2 previousBodyCenter) {
		double[] rayHits = new double[rayDirections.length];

		List<RaycastResult> results = new ArrayList<>();

		for (int rayIdx = 0; rayIdx < rayDirections.length; rayIdx++) {
			double direction = rayDirections[rayIdx];

			direction += head.getWorldCenter().subtract(previousBodyCenter).getDirection() ;
			Ray ray = new Ray(head.getWorldCenter(), direction);
			rayList.add(ray);
			results.clear();
			results = world.raycast(ray, this.rayLength, new DetectFilter<>(true, true, null));
			if (results.isEmpty()) {
				rayHits[rayIdx] = 1d;
			} else {
				rayHits[rayIdx] = results.get(0).getRaycast().getDistance() / rayLength;
			}
		}
		return rayHits;
	}

	public static class RaycastFilter implements Filter, Serializable {

		@Override
		public boolean isAllowed(Filter f) {
			// make sure the given filter is not null
			if (f == null)
				return true;
			// check the type
			return !(f instanceof Prova.ParentFilter);
			// if its not of right type always return true
		}

	}

	public double[] getRayDirections() {
		return rayDirections;
	}

}
