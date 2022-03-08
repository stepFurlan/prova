package provaDyn;

import java.util.ArrayList;
import java.util.List;

import org.dyn4j.geometry.Ray;
import org.dyn4j.world.World;

import com.google.gson.Gson;

public class Snapshot {

	private final double time;
	private final World world;
	private final List<Worm> worms;
	private final WorldTemperature temperature;
	private final List<Ray> rayList;

	public Snapshot(double time, World world, List<Worm> worms, WorldTemperature temperature, List<Ray> rayList) {
		this.time = time;
		Gson gson = new Gson();
		this.world = gson.fromJson(gson.toJson(world), World.class);
		this.worms = new ArrayList<>();
		for(int i = 0; i < worms.size(); i++) {
			this.worms.add(new Worm(worms.get(i)));
		}
		this.temperature = gson.fromJson(gson.toJson(temperature), WorldTemperature.class);
		this.rayList = new ArrayList<>();
		for(int i = 0; i < rayList.size(); i++) {
			this.rayList.add(gson.fromJson(gson.toJson(rayList.get(i)), Ray.class));
		}
	}

	public double getTime() {
		return this.time;
	}

	public World getWorld() {
		return this.world;
	}

	public List<Worm> getWorms() {
		return this.worms;
	}

	public WorldTemperature getTemperature() {
		return this.temperature;
	}

	public List<Ray> getRayList() {
		return this.rayList;
	}

	public int compareTo(Snapshot other) {
		return Double.compare(time, other.time);
	}
}
