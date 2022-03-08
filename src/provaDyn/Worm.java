package provaDyn;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.dyn4j.dynamics.joint.DistanceJoint;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Mass;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.samples.framework.SimulationBody;
import org.dyn4j.samples.framework.SimulationFrame;
import org.dyn4j.world.World;

public class Worm {

	// # of circles per worm
	private final int numCircles;
	// color of the worm
	private final Color HsbColor;
	// # of mini rectangles that form a flagella
	private final int numFlagelli;
	// height of each mini rectangle
	private final double flagelliLength;

	private int generation;

	private Color tempColor;

	private double currentEnergy;

	private int[] currentFoodItems;

	private double currentClosestItemDistance;

	private double currentTouchRight;

	private double currentTouchLeft;

	private Vector2 currentHeadPosition;
	private Vector2 currentFirstBodyPosition;

	private Lidar lidarSensor;

	private Touch touchSensorRight;
	private Touch touchSensorLeft;

	private Nose noseSensor;

	private double xCoord;
	private double yCoord;

	private List<SimulationBody> allBodies;
	private List<SimulationBody> allFlagelsRight;
	private List<SimulationBody> allFlagelsLeft;
	private List<SimulationBody> allFlagels;
	private List<SimulationBody> bodies;
	// creation of flagelli of the worm
	private List<List<SimulationBody>> flagelsRight = new ArrayList<>();
	private List<List<SimulationBody>> flagelsLeft = new ArrayList<>();
	private List<SimulationBody> flagelListRight;
	private List<SimulationBody> flagelListLeft;

	private World world;

	final double ballRadius = 1.5;
	// 0.126 oz/in^3 = 217.97925 kg/m^3
	final double ballDensity = 3.97925;
	final double ballFriction = 8.08;
	final double ballRestitution = 0.8;

	public Worm(int circles, Color color, int flagelli, double lenFlagelli, World world, double xCoord, double yCoord,
			int gen) {
		this.generation = gen;
		this.numCircles = circles;
		this.HsbColor = color;
		this.tempColor = color;
		this.numFlagelli = flagelli;
		this.flagelliLength = lenFlagelli;
		this.lidarSensor = new Lidar();
		this.noseSensor = new Nose();
		this.currentEnergy = 100;
		this.currentFoodItems = new int[] { 0, 0 };
		this.flagelListRight = new ArrayList<>();
		this.flagelListLeft = new ArrayList<>();
		this.allFlagelsRight = new ArrayList<>();
		this.allFlagelsLeft = new ArrayList<>();
		this.allFlagels = new ArrayList<>();
		this.bodies = new ArrayList<>();
		this.world = world;
		this.xCoord = xCoord;
		this.yCoord = yCoord;
		assemble();
	}
	
	public Worm(Worm other) {
		this.numCircles = other.getCircles();
		this.HsbColor = other.getGenoColor();
		this.numFlagelli = other.getFlagelli();
		this.flagelliLength = other.getFlagelliLength();
		this.currentEnergy = other.getCurrentEnergy();
		this.currentHeadPosition = other.getBodies().get(0).getWorldCenter();
		this.currentFirstBodyPosition = other.getBodies().get(1).getWorldCenter();
	}

	private void assemble() {

		for (int k = 0; k < this.numFlagelli; k++) {
			List<SimulationBody> flagelListRight = new ArrayList<>();
			List<SimulationBody> flagelListLeft = new ArrayList<>();
			for (int i = 0; i < numCircles; i++) {
				flagelListRight.add(new SimulationBody(this.HsbColor));
				flagelListLeft.add(new SimulationBody(this.HsbColor));
			}
			flagelsRight.add(flagelListRight);
			flagelsLeft.add(flagelListLeft);
		}

		for (int i = 0; i < numCircles; i++) {
			if (i == 0) {
				bodies.add(new SimulationBody(Color.getHSBColor(0.1f, 1.0f, 0f)));
			} else {
				bodies.add(new SimulationBody(this.HsbColor));
			}
		}

		// double[] minTranslations = new double[Prova.worms.size()];
		// double[] maxTranslations = new double[Prova.worms.size()];
		double transCircles[] = new double[numCircles];
		transCircles[0] = 1.0;
		for (int k = 1; k < transCircles.length; k++) {
			transCircles[k] = transCircles[k - 1] + 2.0;
		}
		/*
		 * minTranslations[0] = transCircles[0]; maxTranslations[0] = 0; for (int i = 1;
		 * i < minTranslations.length; i++) { minTranslations[i] = minTranslations[i -
		 * 1] + 10.0; maxTranslations[i] = maxTranslations[i - 1] + 10.0; }
		 */

		for (SimulationBody body : bodies) {
			// double trans1 = Math.random() * (maxTranslations[j] - minTranslations[j] + 1)
			// + minTranslations[j];
			// double trans2 = Math.random() * (maxTranslations[j] - minTranslations[j] + 1)
			// + minTranslations[j];
			body.addFixture(Geometry.createCircle(ballRadius), ballDensity, ballFriction, ballRestitution);
			// body.setMass(MassType.NORMAL);
			body.setMass(new Mass(body.getWorldCenter(), 10, 20));
			body.translate(transCircles[bodies.indexOf(body)] + xCoord, yCoord);
			body.setLinearDamping(0.5);
			body.setAngularDamping(0.1);
			if (bodies.indexOf(body) == 0) {
				SimulationFrame.wormsHeads.add(body);
			}
			this.world.addBody(body);
			if (bodies.indexOf(body) != 0) {
				// allBodies.add(body);
			}
			body.setUserData(this);
		}

		// setting fixtures for the flagelli and adding them to the world
		for (List<SimulationBody> flagListRight : flagelsRight) {
			if (flagelsRight.indexOf(flagListRight) == 0) {
				for (SimulationBody flag : flagListRight) {
					flag.addFixture(new Rectangle(0.1, this.flagelliLength));
					// flag.setMass(MassType.NORMAL);
					flag.setMass(new Mass(flag.getWorldCenter(), 0.2, 2));
					flag.translate(bodies.get(flagListRight.indexOf(flag)).getWorldCenter().x + ballRadius,
							bodies.get(flagListRight.indexOf(flag)).getWorldCenter().y + ballRadius);
					flag.setLinearDamping(0.4);
					flag.setAngularDamping(0.1);
					this.world.addBody(flag);

					flag.setUserData(this);
				}
			} else {
				for (SimulationBody flag : flagListRight) {
					flag.addFixture(new Rectangle(0.1, this.flagelliLength));
					// flag.setMass(MassType.NORMAL);
					flag.setMass(new Mass(flag.getWorldCenter(), 0.2, 2));
					flag.translate(bodies.get(flagListRight.indexOf(flag)).getWorldCenter().x + ballRadius,
							flagelsRight.get(flagelsRight.indexOf(flagListRight) - 1).get(flagListRight.indexOf(flag))
									.getWorldCenter().y + ballRadius);
					flag.setLinearDamping(0.4);
					flag.setAngularDamping(0.1);
					this.world.addBody(flag);

					flag.setUserData(this);
					if (flagelsRight.indexOf(flagListRight) == flagelsRight.size() - 1) {
						allFlagelsRight.add(flag);
						allFlagels.add(flag);
					}
				}
			}
		}

		for (List<SimulationBody> flagListLeft : flagelsLeft) {
			if (flagelsLeft.indexOf(flagListLeft) == 0) {
				for (SimulationBody flag : flagListLeft) {
					flag.addFixture(new Rectangle(0.1, this.flagelliLength));
					// flag.setMass(MassType.NORMAL);
					flag.setMass(new Mass(flag.getWorldCenter(), 0.2, 2));
					flag.translate(bodies.get(flagListLeft.indexOf(flag)).getWorldCenter().x - ballRadius,
							bodies.get(flagListLeft.indexOf(flag)).getWorldCenter().y - ballRadius);
					flag.setLinearDamping(0.4);
					flag.setAngularDamping(0.1);
					this.world.addBody(flag);

					flag.setUserData(this);
				}
			} else {
				for (SimulationBody flag : flagListLeft) {
					flag.addFixture(new Rectangle(0.1, this.flagelliLength));
					// flag.setMass(MassType.NORMAL);
					flag.setMass(new Mass(flag.getWorldCenter(), 0.2, 2));
					flag.translate(bodies.get(flagListLeft.indexOf(flag)).getWorldCenter().x - ballRadius,
							flagelsLeft.get(flagelsLeft.indexOf(flagListLeft) - 1).get(flagListLeft.indexOf(flag))
									.getWorldCenter().y - ballRadius);
					flag.setLinearDamping(0.4);
					flag.setAngularDamping(0.1);
					this.world.addBody(flag);

					flag.setUserData(this);
					if (flagelsLeft.indexOf(flagListLeft) == flagelsLeft.size() - 1) {
						allFlagelsLeft.add(flag);
						allFlagels.add(flag);
					}
				}
			}
		}

		// allBodies.add(bodies.get(0));
		Prova.listOfAllFlagelsList.add(allFlagels);
		// Prova.listOfAllBodiesList.add(allBodies);
		// Prova.listOfAllBodiesList.add(allBodies);

		// creation of joints for the bodies and flagelli and adding them to the world
		for (SimulationBody body : bodies) {
			if (bodies.indexOf(body) < numCircles - 1) {
				if (bodies.indexOf(body) == 0) {
					DistanceJoint<SimulationBody> jointHead1 = new DistanceJoint<SimulationBody>(body,
							bodies.get(bodies.indexOf(body) + 1), body.getWorldCenter().add(new Vector2(0, 1.5)),
							bodies.get(bodies.indexOf(body) + 1).getWorldCenter());
					DistanceJoint<SimulationBody> jointHead2 = new DistanceJoint<SimulationBody>(body,
							bodies.get(bodies.indexOf(body) + 1), body.getWorldCenter().add(new Vector2(0, -1.5)),
							bodies.get(bodies.indexOf(body) + 1).getWorldCenter());
					this.world.addJoint(jointHead1);
					this.world.addJoint(jointHead2);
					RevoluteJoint<SimulationBody> jointRevoluteHead = new RevoluteJoint<SimulationBody>(body,
							bodies.get(bodies.indexOf(body) + 1), body.getWorldCenter());
					this.world.addJoint(jointRevoluteHead);
				} else {
					DistanceJoint<SimulationBody> joint1 = new DistanceJoint<SimulationBody>(body,
							bodies.get(bodies.indexOf(body) + 1), body.getWorldCenter(),
							bodies.get(bodies.indexOf(body) + 1).getWorldCenter());
					this.world.addJoint(joint1);
				}

			}

			for (List<SimulationBody> flagListRight : flagelsRight) {
				if (flagelsRight.indexOf(flagListRight) == 0) {
					DistanceJoint<SimulationBody> flagelJoint = new DistanceJoint<SimulationBody>(body,
							flagListRight.get(bodies.indexOf(body)),
							body.getWorldCenter().add(new Vector2(0, ballRadius)),
							flagListRight.get(bodies.indexOf(body)).getWorldCenter().add(
									new Vector2(0, -this.flagelliLength / 2 - (this.flagelliLength * (0.8 / 100)))));
					flagelJoint.setDistance(0.1);
					this.world.addJoint(flagelJoint);
				} else {
					DistanceJoint<SimulationBody> flagelJoint = new DistanceJoint<SimulationBody>(
							flagelsRight.get(flagelsRight.indexOf(flagListRight) - 1).get(bodies.indexOf(body)),
							flagListRight.get(bodies.indexOf(body)),
							flagelsRight.get(flagelsRight.indexOf(flagListRight) - 1).get(bodies.indexOf(body))
									.getWorldCenter()
									.add(new Vector2(0, this.flagelliLength / 2 - (this.flagelliLength * (0.8 / 100)))),
							flagListRight.get(bodies.indexOf(body)).getWorldCenter().add(
									new Vector2(0, -this.flagelliLength / 2 - (this.flagelliLength * (0.8 / 100)))));
					flagelJoint.setDistance(0.1);
					this.world.addJoint(flagelJoint);
				}
			}

			for (List<SimulationBody> flagListLeft : flagelsLeft) {
				if (flagelsLeft.indexOf(flagListLeft) == 0) {
					DistanceJoint<SimulationBody> flagelJoint = new DistanceJoint<SimulationBody>(body,
							flagListLeft.get(bodies.indexOf(body)),
							body.getWorldCenter().add(new Vector2(0, -ballRadius)),
							flagListLeft.get(bodies.indexOf(body)).getWorldCenter().add(
									new Vector2(0, this.flagelliLength / 2 - (this.flagelliLength * (0.8 / 100)))));
					flagelJoint.setDistance(0.1);
					this.world.addJoint(flagelJoint);
				} else {
					DistanceJoint<SimulationBody> flagelJoint = new DistanceJoint<SimulationBody>(
							flagelsLeft.get(flagelsLeft.indexOf(flagListLeft) - 1).get(bodies.indexOf(body)),
							flagListLeft.get(bodies.indexOf(body)),
							flagelsLeft.get(flagelsLeft.indexOf(flagListLeft) - 1).get(bodies.indexOf(body))
									.getWorldCenter()
									.add(new Vector2(0,
											-this.flagelliLength / 2 - (this.flagelliLength * (0.8 / 100)))),
							flagListLeft.get(bodies.indexOf(body)).getWorldCenter().add(
									new Vector2(0, this.flagelliLength / 2 - (this.flagelliLength * (0.8 / 100)))));
					flagelJoint.setDistance(0.1);
					this.world.addJoint(flagelJoint);
				}
			}

		}

		this.touchSensorRight = new Touch(this.allFlagelsRight);
		this.touchSensorLeft = new Touch(this.allFlagelsLeft);

	}
	
	public World getWorld() {
		return this.getWorld();
	}

	public Color getGenoColor() {
		return this.HsbColor;
	}

	public int getCircles() {
		return this.numCircles;
	}

	public Color getColor() {
		return this.tempColor;
	}

	public int getFlagelli() {
		return this.numFlagelli;
	}

	public double getFlagelliLength() {
		return this.flagelliLength;
	}

	public Vector2 getHeadPosition() {
		return this.currentHeadPosition;
	}

	public void setHeadPosition(Vector2 position) {
		this.currentHeadPosition = position;
	}

	public Lidar getLidar() {
		return this.lidarSensor;
	}

	public Touch getTouchRight() {
		return this.touchSensorRight;
	}

	public Touch getTouchLeft() {
		return this.touchSensorLeft;
	}

	public Nose getNose() {
		return this.noseSensor;
	}

	public double getCurrentEnergy() {
		return this.currentEnergy;
	}

	public List<SimulationBody> getBodies() {
		return this.bodies;
	}

	public void setEnergy(int newEnergy) {
		this.currentEnergy = newEnergy;
	}

	public void setCurrentFoodItems(int[] food) {
		this.currentFoodItems = food;
	}

	public int[] getCurrentFoodItems() {
		return this.currentFoodItems;
	}

	public void setCurrentClosestItemDistance(double distance) {
		this.currentClosestItemDistance = distance;
	}

	public double getCurrentClosestItemDistance() {
		return this.currentClosestItemDistance;
	}

	public void setCurrentTouchRight(double touch) {
		this.currentTouchRight = touch;
	}

	public void setCurrentTouchLeft(double touch) {
		this.currentTouchLeft = touch;
	}

	public double getCurrentTouchRight() {
		return this.currentTouchRight;
	}

	public double getCurrentTouchLeft() {
		return this.currentTouchLeft;
	}

	public void setGeneration(int gen) {
		this.generation = gen;
	}

	public int getGeneration() {
		return this.generation;
	}

	public void setColor(Color color) {
		for (SimulationBody body : this.bodies) {
			if (this.bodies.indexOf(body) != 0) {
				body.setColor(color);
			}
		}
		this.tempColor = color;
	}

	public double getAngle() {
		Vector2 vec = new Vector2(this.bodies.get(1).getWorldCenter().x, this.bodies.get(1).getWorldCenter().y,
				this.bodies.get(0).getWorldCenter().x, this.bodies.get(0).getWorldCenter().y);
		return vec.getDirection();
		// return
		// this.bodies.get(0).getWorldCenter().subtract(this.bodies.get(1).getWorldCenter()).getDirection();
	}

	public void kill() {
		for (SimulationBody body : bodies) {
			if (bodies.indexOf(body) == 0) {
				SimulationFrame.wormsHeads.remove(body);
			}
			this.world.removeBody(body);
		}
		for (List<SimulationBody> listFlag : flagelsRight) {
			for (SimulationBody flagel : listFlag) {
				this.world.removeBody(flagel);
			}
		}
		for (List<SimulationBody> listFlag : flagelsLeft) {
			for (SimulationBody flagel : listFlag) {
				this.world.removeBody(flagel);
			}
		}
	}

}
