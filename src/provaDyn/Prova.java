package provaDyn;

import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Point;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.swing.JFrame;
import javax.swing.JLabel;

import org.dyn4j.collision.AbstractCollisionPair;
import org.dyn4j.collision.Filter;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.joint.AngleJoint;
import org.dyn4j.dynamics.joint.DistanceJoint;
import org.dyn4j.dynamics.joint.MotorJoint;
import org.dyn4j.dynamics.joint.PrismaticJoint;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.dynamics.joint.RopeJoint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Capsule;
import org.dyn4j.geometry.Circle;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Polygon;
import org.dyn4j.geometry.Ray;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Slice;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import java.awt.event.MouseAdapter;
import org.dyn4j.samples.framework.SimulationBody;
import org.dyn4j.samples.framework.SimulationFrame;
import org.dyn4j.world.World;
import org.dyn4j.world.result.RaycastResult;

public class Prova extends SimulationFrame {
	private static final long serialVersionUID = 5663760293144882635L;

	// protected List<List<SimulationBody>> wormBodies;

	public static List<Worm> worms;
	private static long last;

	private static long timez;

	private static List<SimulationBody> foodCoordinates = new ArrayList<>();;

	public static List<List<SimulationBody>> listOfAllFlagelsList = new ArrayList<>();
	public static List<List<SimulationBody>> listOfAllBodiesList = new ArrayList<>();

	private final ExecutorService executorService;

	public Prova() {
		super("Prova", 45.0);
		this.executorService = Executors.newFixedThreadPool(6);
	}

	public static void main(String[] args) {
		rayList = new ArrayList<>();
		worms = new ArrayList<>();
		Prova simulation = new Prova();
		last = System.nanoTime();
		simulation.setMousePanningEnabled(false);
		simulation.run();
	}

	public static class ParentFilter implements Filter {

		private final Object parent;

		public ParentFilter(Object parent) {
			this.parent = parent;
		}

		@Override
		public boolean isAllowed(Filter f) {
			if (!(f instanceof ParentFilter)) {
				return true;
			}
			return parent != ((ParentFilter) f).parent;
		}

	}

	public void setOwner(SimulationBody head, List<SimulationBody> bodies) {
		Filter filter;
		filter = new ParentFilter(head);

		for (SimulationBody body : bodies) {
			for (Fixture fixture : body.getFixtures()) {
				fixture.setFilter(filter);
			}
		}
	}

	private boolean checkWormDistance(List<Double> bodiesXPositions, List<Double> bodiesYPositions, int randX,
			int randY, int circles) {
		for (int i = 0; i < bodiesXPositions.size(); i++) {
			int add = 0;
			for (int j = 0; j < circles; j++) {
				if (Math.abs(Math.round(bodiesXPositions.get(i)) - (randX + add)) <= 10
						&& Math.abs(Math.round(bodiesYPositions.get(i)) - (randY)) <= 10) {
					return true;
				}
				add += 2;
			}
		}
		return false;
	}

	private void addWorm(int circles, Color color, int flagelli, double lenFlagelli, int gen) {

		Random random = new Random();
		int randX = random.nextInt(150) - 75;
		int randY = random.nextInt(150) - 75;
		List<Double> bodiesXPositions = new ArrayList<>();
		List<Double> bodiesYPositions = new ArrayList<>();
		if (worms.size() != 0) {
			for (Worm worm : worms) {
				for (SimulationBody body : worm.getBodies()) {
					bodiesXPositions.add(body.getWorldCenter().x);
					bodiesYPositions.add(body.getWorldCenter().y);
				}
			}
		}

		while (checkWormDistance(bodiesXPositions, bodiesYPositions, randX, randY, circles)) {
			random = new Random();
			randX = random.nextInt(150) - 75;
			randY = random.nextInt(150) - 75;
		}
		Worm worm = new Worm(circles, color, flagelli, lenFlagelli, this.world, randX, randY, gen);
		worms.add(worm);
		temperature.setMatrixValue(randY + (int) yWorldDimension / 2, randX + (int) xWorldDimension / 2, 100);
		temperature.setMatrixValue(randY + 1 + (int) yWorldDimension / 2, randX + (int) xWorldDimension / 2, 100);
		temperature.setMatrixValue(randY - 1 + (int) yWorldDimension / 2, randX + (int) xWorldDimension / 2, 100);
		temperature.setMatrixValue(randY + (int) yWorldDimension / 2, randX + 1 + (int) xWorldDimension / 2, 100);
		temperature.setMatrixValue(randY + (int) yWorldDimension / 2, randX - 1 + (int) xWorldDimension / 2, 100);

	}

	private void checkWorldTemperature() {
		for (int i = 0; i < yWorldDimension; i++) {
			for (int j = 0; j < xWorldDimension; j++) {
				if (temperature.getMatrix()[i][j] != 0) {
					Rectangle tempRect = new Rectangle(1.0, 1.0);
					SimulationBody temp = new SimulationBody(Color.orange);
					temp.addFixture(new BodyFixture(tempRect));
					temp.setMass(MassType.INFINITE);
					temp.setEnabled(false);
					temp.translate(j - xWorldDimension / 2, i - yWorldDimension / 2);
					this.world.addBody(temp);
					temp.setUserData(temp);
				}
			}
		}
	}

	protected void initializeWorld() {

		this.world.setGravity(World.ZERO_GRAVITY);
		this.temperature = new WorldTemperature((int) xWorldDimension, (int) yWorldDimension);
		this.temperaturePast = new WorldTemperature((int) xWorldDimension, (int) yWorldDimension);

		// checkWorldTemperature();

		// add floor
		Rectangle floorRect = new Rectangle(xWorldDimension, 1.0);
		SimulationBody floor = new SimulationBody(Color.GRAY);
		floor.addFixture(new BodyFixture(floorRect));
		floor.setMass(MassType.INFINITE);
		floor.translate(0.0, -yWorldDimension / 2);
		this.world.addBody(floor);
		floor.setUserData(floor);

		// add ceiling
		Rectangle ceilingRect = new Rectangle(xWorldDimension, 1.0);
		SimulationBody ceiling = new SimulationBody(Color.GRAY);
		ceiling.addFixture(new BodyFixture(ceilingRect));
		ceiling.setMass(MassType.INFINITE);
		ceiling.translate(0.0, yWorldDimension / 2);
		this.world.addBody(ceiling);
		ceiling.setUserData(ceiling);

		// add right wall
		Rectangle rightWallRect = new Rectangle(1.0, yWorldDimension);
		SimulationBody rightWall = new SimulationBody(Color.GRAY);
		rightWall.addFixture(new BodyFixture(rightWallRect));
		rightWall.setMass(MassType.INFINITE);
		rightWall.translate(xWorldDimension / 2, 0.0);
		this.world.addBody(rightWall);
		rightWall.setUserData(rightWall);

		// add left wall
		Rectangle leftWallRect = new Rectangle(1.0, yWorldDimension);
		SimulationBody leftWall = new SimulationBody(Color.GRAY);
		leftWall.addFixture(new BodyFixture(leftWallRect));
		leftWall.setMass(MassType.INFINITE);
		leftWall.translate(-xWorldDimension / 2, 0.0);
		this.world.addBody(leftWall);
		leftWall.setUserData(leftWall);

		// create worms
		worms = new ArrayList<>();
		addWorm(6, Color.getHSBColor(3.9f, 1.0f, 0.5f), 3, 0.9, 0);
		addWorm(10, Color.getHSBColor(3.7f, 1.0f, 0.5f), 3, 1.2, 0);
		addWorm(14, Color.getHSBColor(0.1f, 1.0f, 0.5f), 5, 0.8, 0);
		addWorm(11, Color.getHSBColor(0.7f, 1.0f, 0.5f), 5, 0.8, 0);
		addWorm(7, Color.getHSBColor(1.1f, 1.0f, 0.5f), 5, 0.5, 0);
		addWorm(9, Color.getHSBColor(0.5f, 1.0f, 0.5f), 6, 0.5, 0);
		addWorm(9, Color.getHSBColor(0.5f, 1.0f, 0.5f), 2, 0.8, 0);
		addWorm(8, Color.getHSBColor(0.5f, 1.0f, 0.5f), 2, 0.8, 0);
		addWorm(14, Color.getHSBColor(0.5f, 1.0f, 0.5f), 2, 1.8, 0);
		addWorm(9, Color.getHSBColor(0.5f, 1.0f, 0.5f), 2, 0.8, 0);
		addWorm(6, Color.getHSBColor((150 / 360) * 255, 1.0f, 0.5f), 2, 1.8, 0);
		addWorm(20, Color.getHSBColor((90 / 360) * 255, 1.0f, 0.5f), 4, 1.0, 0);
		addWorm(5, Color.getHSBColor(0f, 1.0f, 0.5f), 2, 0.8, 0);
		addWorm(8, Color.getHSBColor((60 / 360) * 255, 1.0f, 0.5f), 3, 1.5, 0);

		// List<Vector2> foodCoordinates = new ArrayList<>();
		for (int i = 0; i < 1800; i++) {
			setFood();
		}
		timez = System.nanoTime();

	}

	// set food random
	private void setFood() {
		Circle foodRect = new Circle(0.5);
		SimulationBody food = new SimulationBody(Color.GREEN);
		food.addFixture(new BodyFixture(foodRect));
		food.setMass(MassType.NORMAL);
		food.setEnabled(true);
		Random random = new Random();
		int xRand = random.nextInt(418) - 209;
		int yRand = random.nextInt(238) - 119;
		foodCoordinates.add(food);
		food.translate(xRand, yRand);
		food.setLinearDamping(1.9);
		food.setAngularDamping(0.9);
		this.world.addBody(food);
		// food.setUserData(food);
		// System.out.println(food.getUserData());
	}

	// set food where a worm dies
	private void setFoodAfterDeath(double xCoord, double yCoord) {
		Circle foodRect = new Circle(0.5);
		SimulationBody food = new SimulationBody(Color.GREEN);
		food.addFixture(new BodyFixture(foodRect));
		food.setMass(MassType.NORMAL);
		food.setEnabled(true);
		foodCoordinates.add(food);
		food.translate(xCoord, yCoord);
		food.setLinearDamping(1.9);
		food.setAngularDamping(0.9);
		this.world.addBody(food);
		// food.setUserData(food);
		// System.out.println(food.getUserData());
	}

	public World getWorld() {
		return world;
	}

	private SimulationBody checkForFood(SimulationBody body) {
		List<SimulationBody> inContactBodies = this.world.getInContactBodies(body, false);
		for (SimulationBody inContactBody : inContactBodies) {
			Object userData = inContactBody.getUserData();
			if (userData == null) {
				return inContactBody;
			}
		}
		return body;
	}

	private static double smallestNumber(double[] array) {
		double smallest = Double.MAX_VALUE;
		int index = 0;
		while (index < array.length) {
			// check if smallest is greater than element
			if (smallest > array[index]) {
				// update smallest
				smallest = array[index];
			}
			index++;
		}
		return smallest;
	}

	private void evolve(Worm worm) {
		Color color = worm.getGenoColor();
		int numFlagelli = worm.getFlagelli();
		double flagelliLength = worm.getFlagelliLength();
		int numCircles = worm.getCircles();
		Vector2 headPosition = worm.getHeadPosition();
		int gen = worm.getGeneration();
		killWorm(worm);
		for (int i = 0; i < numCircles; i++) {
			double randX = Math.random() * (2.5 + 2.5 + 1) - 2.5;
			double randY = Math.random() * (2.5 + 2.5 + 1) - 2.5;
			setFoodAfterDeath(headPosition.x + randX, headPosition.y + randY);
		}
		addWorm(numCircles, color, numFlagelli, flagelliLength, gen + 1);
	}

	private void killWorm(Worm worm) {
		worm.kill();
		worms.remove(worm);
	}

	private void setTemperature(double xx, double yy, double value) {
		int y = (int) Math.round(xx) + (int) xWorldDimension / 2;
		int x = (int) Math.round(yy) + (int) yWorldDimension / 2;
		double[][] tempMatr = temperature.getMatrix();
		if (tempMatr[x][y] + value <= 4) {
			temperature.setMatrixValue(x, y, tempMatr[x][y] + value);
		}
		if (tempMatr[x + 2][y] + value <= 4) {
			temperature.setMatrixValue(x + 2, y, tempMatr[x + 2][y] + value);
		}
		if (tempMatr[x - 1][y] + value <= 4) {
			temperature.setMatrixValue(x - 1, y, tempMatr[x - 1][y] + value);
		}
		if (tempMatr[x - 2][y] + value <= 4) {
			temperature.setMatrixValue(x - 2, y, tempMatr[x - 2][y] + value);
		}
		if (tempMatr[x][y + 2] + value <= 4) {
			temperature.setMatrixValue(x, y + 2, tempMatr[x][y + 2] + value);
		}
		if (tempMatr[x][y - 1] + value <= 4) {
			temperature.setMatrixValue(x, y - 1, tempMatr[x][y - 1] + value);
		}
		if (tempMatr[x][y - 2] + value <= 4) {
			temperature.setMatrixValue(x, y - 2, tempMatr[x][y - 2] + value);
		}
		if (tempMatr[x + 1][y] + value <= 4) {
			temperature.setMatrixValue(x + 1, y, tempMatr[x + 1][y] + value);
		}
		if (tempMatr[x + 1][y + 1] + value <= 4) {
			temperature.setMatrixValue(x + 1, y + 1, tempMatr[x + 1][y + 1] + value);
		}
		if (tempMatr[x + 1][y - 1] + value <= 4) {
			temperature.setMatrixValue(x + 1, y - 1, tempMatr[x + 1][y - 1] + value);
		}
		if (tempMatr[x + 1][y - 2] + value <= 4) {
			temperature.setMatrixValue(x + 1, y - 2, tempMatr[x + 1][y - 2] + value);
		}
		if (tempMatr[x + 1][y + 2] + value <= 4) {
			temperature.setMatrixValue(x + 1, y + 2, tempMatr[x + 1][y + 2] + value);
		}
		if (tempMatr[x][y + 1] + value <= 4) {
			temperature.setMatrixValue(x, y + 1, tempMatr[x][y + 1] + value);
		}
		if (tempMatr[x - 1][y + 1] + value <= 4) {
			temperature.setMatrixValue(x - 1, y + 1, tempMatr[x - 1][y + 1] + value);
		}
		if (tempMatr[x - 1][y - 1] + value <= 4) {
			temperature.setMatrixValue(x - 1, y - 1, tempMatr[x - 1][y - 1] + value);
		}
		if (tempMatr[x - 1][y - 2] + value <= 4) {
			temperature.setMatrixValue(x - 1, y - 2, tempMatr[x - 1][y - 2] + value);
		}
		if (tempMatr[x - 1][y + 2] + value <= 4) {
			temperature.setMatrixValue(x - 1, y + 2, tempMatr[x - 1][y + 2] + value);
		}

		// temperature.setMatrixValue(x, y, value);
		// temperature.setMatrixValue(x, y, value);

	}

	private void setTemperature2() {
		double[][] tempMatr = temperature.getMatrix();
		for (int i = 1; i < yWorldDimension - 1; i++) {
			for (int j = 1; j < xWorldDimension - 1; j++) {
				temperature.setMatrixValue(i, j,
						((tempMatr[i][j] + tempMatr[i][j + 1] + tempMatr[i][j - 1] + tempMatr[i - 1][j - 1]
								+ tempMatr[i - 1][j] + tempMatr[i - 1][j + 1] + tempMatr[i + 1][j - 1]
								+ tempMatr[i + 1][j] + tempMatr[i + 1][j + 1]) / 9) * 0.99);
			}
		}

	}

	public static double getMaxValue(double[][] numbers) {
		double maxValue = numbers[0][0];
		for (int j = 0; j < numbers.length; j++) {
			for (int i = 0; i < numbers[j].length; i++) {
				if (numbers[j][i] > maxValue) {
					maxValue = numbers[j][i];
				}
			}
		}
		return maxValue;
	}

	public static double getMinValue(double[][] numbers) {
		double minValue = numbers[0][0];
		for (int j = 0; j < numbers.length; j++) {
			for (int i = 0; i < numbers[j].length; i++) {
				if (numbers[j][i] < minValue) {
					minValue = numbers[j][i];
				}
			}
		}
		return minValue;
	}

	protected void handleEvents() {
		 System.out.println(countIterations);
		countIterations++;
		setTemperature2();
		for (Worm worm : worms) {
			try {
				// this.executorService.execute(()->{
				// System.out.println(worm.getGeneration());

				/*
				 * if(worm.getGeneration() == 1) { long time = System.nanoTime();
				 * System.out.println("First evolution! Evolution time: "+ (double) (time-timez)
				 * / NANO_TO_BASE); System.exit(0); }
				 */
				// System.out.println(worm.getCurrentEnergy());
				for (SimulationBody bodyT : worm.getBodies()) {
					temperature.setMatrixValue((int) Math.round(bodyT.getWorldCenter().y + yWorldDimension / 2),
							(int) Math.round(bodyT.getWorldCenter().x + xWorldDimension / 2), 150);

				}

				// check worm energy and in case of energy <= 0 evolve the worm
				if (worm.getCurrentEnergy() <= 0) {
					evolve(worm);
					break;
				}

				worm.setCurrentTouchRight(worm.getTouchRight().touch(this.world)[0]);
				worm.setCurrentTouchLeft(worm.getTouchLeft().touch(this.world)[0]);

				// int headIndex = wormsHeads.indexOf(head);
				worm.setHeadPosition(worm.getBodies().get(0).getWorldCenter());
				double[] res = new double[worm.getLidar().getRayDirections().length];
				res = worm.getLidar().view(worm.getBodies().get(0), this.world, rayList,
						worm.getBodies().get(1).getWorldCenter());
				List<Double> resList = new ArrayList<>();
				for (int a = 0; a < res.length; a++) {
					resList.add(res[a]);
				}

				worm.setCurrentClosestItemDistance(smallestNumber(res));

				// check if a food item is eaten; if yes increment energy and add another food
				// item
				SimulationBody contactBody = checkForFood(worm.getBodies().get(0));
				if (contactBody != worm.getBodies().get(0)) {
					if (worm.getCurrentEnergy() <= 97)
						worm.setEnergy((int) worm.getCurrentEnergy() + 3);
					this.world.removeBody(contactBody);
					foodCoordinates.remove(contactBody);
					setFood();
				}

				// set the current number of food items smelled

				worm.setCurrentFoodItems(worm.getNose().smell2(worm.getBodies().get(0), foodCoordinates,
						worm.getBodies().get(1).getWorldCenter()));

				// set the current head position
				if (worms.indexOf(worm) == 0) {
					worm.setHeadPosition(worm.getBodies().get(0).getWorldCenter());
				}

				// apply random force to move the worm
				Random random = new Random();
				int xRand = random.nextInt(5000) - 2500;
				int yRand = random.nextInt(5000) - 2500;
				worm.getBodies().get(0).applyForce(new Force(new Vector2(xRand, yRand)));
				// });
			} catch (Exception e) {
				System.err.printf("IO error: %s", e);
			}
		}

		long time = System.nanoTime();
		long diff = time - last;
		double elapsedTime = (double) diff / NANO_TO_BASE;
		float[] hsbvals = new float[3];

		// if(elapsedTime >= 5.0 && elapsedTime <= 5.5){
		if (countIterations >= 120) {
			// temperaturePast = temperature;

			/*
			 * for(int i = 0; i < yWorldDimension; i++) { for(int j = 0; j <
			 * xWorldDimension; j++) { if(temperature.getMatrix()[i][j] > 0) {
			 * temperature.getMatrix()[i][j] = temperature.getMatrix()[i][j]-0.5; } } }
			 */
			last = time;
			List<List<SimulationBody>> wormBodies = new ArrayList<>();
			for (Worm worm : worms) {
				wormBodies.add(worms.get(worms.indexOf(worm)).getBodies());
				worm.setEnergy((int) worm.getCurrentEnergy() - 3);
				hsbvals = worm.getColor().RGBtoHSB(worm.getColor().getRed(), worm.getColor().getGreen(),
						worm.getColor().getBlue(), hsbvals);
				worm.setColor(Color.getHSBColor(hsbvals[0], (float) (worm.getCurrentEnergy() * 0.01), hsbvals[2]));
			}
			for (List<SimulationBody> bodyList : wormBodies) {
				for (SimulationBody body : bodyList) {
					hsbvals = body.getColor().RGBtoHSB(body.getColor().getRed(), body.getColor().getGreen(),
							body.getColor().getBlue(), hsbvals);

				}
			}
			countIterations = 0;
		}

	}

}
