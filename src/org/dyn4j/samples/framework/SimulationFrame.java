/*
 * Copyright (c) 2010-2021 William Bittle  http://www.dyn4j.org/
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted 
 * provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice, this list of conditions 
 *     and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
 *     and the following disclaimer in the documentation and/or other materials provided with the 
 *     distribution.
 *   * Neither the name of dyn4j nor the names of its contributors may be used to endorse or 
 *     promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.dyn4j.samples.framework;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.Toolkit;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Line2D.Double;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferStrategy;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;

import org.dyn4j.collision.AxisAlignedBounds;
import org.dyn4j.collision.Bounds;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.Settings;
import org.dyn4j.dynamics.contact.ContactConstraint;
import org.dyn4j.dynamics.contact.SolvedContact;
import org.dyn4j.dynamics.joint.DistanceJoint;
import org.dyn4j.dynamics.joint.Joint;
import org.dyn4j.dynamics.joint.PinJoint;
import org.dyn4j.geometry.AABB;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Ray;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.samples.framework.input.ToggleStateKeyboardInputHandler;
import org.dyn4j.samples.framework.input.CodeExporter;
import org.dyn4j.samples.framework.input.MousePanningInputHandler;
import org.dyn4j.samples.framework.input.MousePickingInputHandler;
import org.dyn4j.samples.framework.input.MouseZoomInputHandler;
import org.dyn4j.world.World;
import org.dyn4j.world.WorldCollisionData;

import provaDyn.Lidar;
import provaDyn.Nose;
import provaDyn.Prova;
import provaDyn.WorldTemperature;
import provaDyn.Worm;

/**
 * A simple framework for building samples.
 * 
 * @version 4.1.1
 * @since 3.2.0
 */
public abstract class SimulationFrame extends JFrame {
	/** The serial version id */
	private static final long serialVersionUID = 7659608187025022915L;

	/** The conversion factor from nano to base */
	public static final double NANO_TO_BASE = 1.0e9;

	/** The canvas to draw to */
	protected final Canvas canvas;

	/** The dynamics engine */
	protected final World<SimulationBody> world;

	protected static List<Ray> rayList;

	public static List<SimulationBody> wormsHeads = new ArrayList<>();;

	// stop/pause

	/** True if the simulation is exited */
	private boolean stopped;

	/** The time stamp for the last iteration */
	private long last;

	// camera

	protected final Camera camera;

	protected final double xWorldDimension = 420;
	protected final double yWorldDimension = 240;

	protected WorldTemperature temperature;
	protected WorldTemperature temperaturePast;

	// interaction (mouse/keyboard)

	private final ToggleStateKeyboardInputHandler paused;
	private final ToggleStateKeyboardInputHandler step;

	private final MousePickingInputHandler picking;
	private final MousePanningInputHandler panning;
	private final MouseZoomInputHandler zoom;

	private final ToggleStateKeyboardInputHandler renderContacts;
	private final ToggleStateKeyboardInputHandler renderBodyAABBs;
	private final ToggleStateKeyboardInputHandler renderBodyRotationRadius;
	private final ToggleStateKeyboardInputHandler renderFixtureAABBs;
	private final ToggleStateKeyboardInputHandler renderFixtureRotationRadius;
	private final ToggleStateKeyboardInputHandler renderLidar;
	private final ToggleStateKeyboardInputHandler renderSmellRadius;
	private final ToggleStateKeyboardInputHandler renderTemperature;
	private final ToggleStateKeyboardInputHandler renderSensorValues;

	protected long countIterations;

	/**
	 * Constructor.
	 * <p>
	 * By default creates a 800x600 canvas.
	 * 
	 * @param name  the frame name
	 * @param scale the pixels per meter scale factor
	 */
	public SimulationFrame(String name, double scale) {
		super(name);
		this.countIterations = 0;
		this.camera = new Camera();
		this.camera.scale = scale;

		// create the world
		this.world = new World<SimulationBody>();

		// this.world.getSettings().setStepFrequency(0.0026);
		// setup the JFrame
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		// add a window listener
		this.addWindowListener(new WindowAdapter() {
			/*
			 * (non-Javadoc)
			 * 
			 * @see java.awt.event.WindowAdapter#windowClosing(java.awt.event.WindowEvent)
			 */
			@Override
			public void windowClosing(WindowEvent e) {
				// before we stop the JVM stop the simulation
				stop();
				super.windowClosing(e);
			}
		});

		// create the size of the window
		// Dimension size = new Dimension(800, 600);
		Dimension size = Toolkit.getDefaultToolkit().getScreenSize();

		// create a canvas to paint to
		this.canvas = new Canvas();
		this.canvas.setPreferredSize(size);
		this.canvas.setMinimumSize(size);
		this.canvas.setMaximumSize(size);
		this.canvas.setBackground(Color.getHSBColor(0.172f, 0.2f, 0.8f));

		// add the canvas to the JFrame
		this.add(this.canvas);

		// make the JFrame not resizable
		// (this way I dont have to worry about resize events)
		this.setResizable(true);

		// size everything
		this.pack();

		this.canvas.requestFocus();

		// install input handlers
		this.picking = new MousePickingInputHandler(this.canvas, this.camera, this.world);
		this.picking.install();
		this.panning = new MousePanningInputHandler(this.canvas, this.camera);
		this.panning.install();
		// panning and picking are dependent
		this.picking.getDependentBehaviors().add(this.panning);
		this.panning.getDependentBehaviors().add(this.picking);

		this.zoom = new MouseZoomInputHandler(this.canvas, this.camera, MouseEvent.BUTTON1);
		this.zoom.install();

		this.paused = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_SPACE);
		this.step = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_ENTER);
		this.renderContacts = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_C);
		this.renderBodyAABBs = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_B);
		this.renderBodyRotationRadius = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_B);
		this.renderLidar = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_L);
		this.renderFixtureAABBs = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_F);
		this.renderFixtureRotationRadius = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_F);
		this.renderSmellRadius = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_N);
		this.renderTemperature = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_T);
		this.renderSensorValues = new ToggleStateKeyboardInputHandler(this.canvas, KeyEvent.VK_S);

		this.paused.install();
		this.step.install();
		this.step.setDependentBehaviorsAdditive(true);
		this.step.getDependentBehaviors().add(this.paused);
		this.renderContacts.install();
		this.renderBodyAABBs.install();
		this.renderBodyRotationRadius.install();
		this.renderFixtureAABBs.install();
		this.renderFixtureRotationRadius.install();
		this.renderLidar.install();
		this.renderSmellRadius.install();
		this.renderTemperature.install();
		this.renderSensorValues.install();

		// setup the world
		this.initializeWorld();
	}

	/**
	 * Creates game objects and adds them to the world.
	 */
	protected abstract void initializeWorld();

	/**
	 * Start active rendering the simulation.
	 * <p>
	 * This should be called after the JFrame has been shown.
	 */
	private void start() {
		// initialize the last update time
		this.last = System.nanoTime();
		// don't allow AWT to paint the canvas since we are
		this.canvas.setIgnoreRepaint(true);
		// enable double buffering (the JFrame has to be
		// visible before this can be done)
		this.canvas.createBufferStrategy(2);
		// run a separate thread to do active rendering
		// because we don't want to do it on the EDT
		Thread thread = new Thread() {
			public void run() {
				// perform an infinite loop stopped
				// render as fast as possible
				while (!isStopped()) {
					gameLoop();
					// you could add a Thread.yield(); or
					// Thread.sleep(long) here to give the
					// CPU some breathing room
					try {
						// Thread.yield();
						Thread.sleep(5);
					} catch (InterruptedException e) {
					}
				}
			}
		};
		// set the game loop thread to a daemon thread so that
		// it cannot stop the JVM from exiting
		thread.setDaemon(true);
		// start the game loop
		thread.start();
	}

	/**
	 * The method calling the necessary methods to update the game, graphics, and
	 * poll for input.
	 */
	private void gameLoop() {
		// get the graphics object to render to
		Graphics2D g = (Graphics2D) this.canvas.getBufferStrategy().getDrawGraphics();

		// by default, set (0, 0) to be the center of the screen with the positive x
		// axis
		// pointing right and the positive y axis pointing up
		this.transform(g);

		// reset the view
		this.clear(g);

		// get the current time
		long time = System.nanoTime();
		// get the elapsed time from the last iteration
		long diff = time - this.last;
		// set the last time
		this.last = time;
		// convert from nanoseconds to seconds
		double elapsedTime = (double) diff / NANO_TO_BASE;

		// render anything about the simulation (will render the World objects)
		AffineTransform tx = g.getTransform();
		g.translate(this.camera.offsetX, this.camera.offsetY);
		this.render(g, elapsedTime);
		g.setTransform(tx);

		// update the World
		if (!this.paused.isActive()) {
			this.world.update(elapsedTime);
		} else if (this.step.isActive()) {
			this.world.step(1);
			this.step.setActive(false);
		}

		double freq = this.world.getSettings().getStepFrequency();
		// System.out.println(freq);

		this.handleEvents();

		// dispose of the graphics object
		g.dispose();

		// blit/flip the buffer
		BufferStrategy strategy = this.canvas.getBufferStrategy();
		if (!strategy.contentsLost()) {
			strategy.show();
		}

		// Sync the display on some systems.
		// (on Linux, this fixes event queue problems)
		Toolkit.getDefaultToolkit().sync();
	}

	/**
	 * Performs any transformations to the graphics.
	 * <p>
	 * By default, this method puts the origin (0,0) in the center of the window and
	 * points the positive y-axis pointing up.
	 * 
	 * @param g the graphics object to render to
	 */
	protected void transform(Graphics2D g) {
		final int w = this.canvas.getWidth();
		final int h = this.canvas.getHeight();

		// before we render everything im going to flip the y axis and move the
		// origin to the center (instead of it being in the top left corner)
		AffineTransform yFlip = AffineTransform.getScaleInstance(1, -1);
		AffineTransform move = AffineTransform.getTranslateInstance(w / 2, -h / 2);
		g.transform(yFlip);
		g.transform(move);
	}

	/**
	 * Clears the previous frame.
	 * 
	 * @param g the graphics object to render to
	 */
	protected void clear(Graphics2D g) {
		final int w = this.canvas.getWidth();
		final int h = this.canvas.getHeight();

		// lets draw over everything with a white background
		g.setColor(Color.getHSBColor(0.172f, 0.2f, 0.8f));
		g.fillRect(-w / 2, -h / 2, w, h);
	}

	/**
	 * Renders the example.
	 * 
	 * @param g           the graphics object to render to
	 * @param elapsedTime the elapsed time from the last update
	 */

	protected void renderSensingParameters(Graphics2D g) {

	}

	protected void render(Graphics2D g, double elapsedTime) {
		g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

		// draw the bounds (if set)
		Bounds bounds = this.world.getBounds();
		if (bounds != null && bounds instanceof AxisAlignedBounds) {
			AxisAlignedBounds aab = (AxisAlignedBounds) bounds;
			AABB aabb = aab.getBounds();
			Rectangle2D.Double ce = new Rectangle2D.Double(aabb.getMinX() * this.camera.scale,
					aabb.getMinY() * this.camera.scale, aabb.getWidth() * this.camera.scale,
					aabb.getHeight() * this.camera.scale);
			g.setColor(new Color(128, 0, 128));
			g.draw(ce);
		}

		if (this.renderTemperature.isActive()) {
			for (int i = 0; i < yWorldDimension; i++) {
				for (int j = 0; j < xWorldDimension; j++) {
					if (temperature.getMatrix()[i][j] <= 0) {

					}
					if (temperature.getMatrix()[i][j] > 0 && temperature.getMatrix()[i][j] < 1) {
						Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale,
								(i - yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale,
								1.0 * this.camera.scale);
						g.setColor(new Color(0.89f, 0.15f, 0.21f, (float) temperature.getMatrix()[i][j]));
					}
					/*
					 * if (temperature.getMatrix()[i][j] >= 1 && temperature.getMatrix()[i][j] < 2)
					 * { Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2)
					 * * this.camera.scale, (i - yWorldDimension / 2) * this.camera.scale, 1.0 *
					 * this.camera.scale, 1.0 * this.camera.scale); g.setColor(new Color(1.0f, 1.0f,
					 * 0.10f, (float) (((temperature.getMatrix()[i][j] - 1.0) / 2)))); //
					 * g.setColor(Color.getHSBColor(1.0472f, (float) temperature.getMatrix()[i][j] -
					 * // 1.0f, 1f)); g.fill(temp); g.draw(temp); } if
					 * (temperature.getMatrix()[i][j] >= 2 && temperature.getMatrix()[i][j] < 3) {
					 * Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) *
					 * this.camera.scale, (i - yWorldDimension / 2) * this.camera.scale, 1.0 *
					 * this.camera.scale, 1.0 * this.camera.scale); g.setColor(new Color(1.0f,
					 * 0.75f, 0.0f, (float) (0.3 + ((temperature.getMatrix()[i][j] - 2.0) / 2))));
					 * // g.setColor(Color.getHSBColor(0.523599f, (float)
					 * temperature.getMatrix()[i][j] // - 2.0f, 1f)); g.fill(temp); g.draw(temp); }
					 * if (temperature.getMatrix()[i][j] >= 3 && temperature.getMatrix()[i][j] < 4)
					 * { Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2)
					 * * this.camera.scale, (i - yWorldDimension / 2) * this.camera.scale, 1.0 *
					 * this.camera.scale, 1.0 * this.camera.scale); g.setColor(new Color(1.0f,
					 * 0.40f, 0.40f, (float) (0.3 + ((temperature.getMatrix()[i][j] - 3.0) / 2))));
					 * // g.setColor(Color.getHSBColor(1.0f, (float)temperature.getMatrix()[i][j] -
					 * // 3.0f, 0.1f)); g.fill(temp); g.draw(temp); }
					 */
					/*
					 * if (temperature.getMatrix()[i][j] >= 4 && temperature.getMatrix()[i][j] < 5)
					 * { Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2)
					 * * this.camera.scale, (i - yWorldDimension / 2) * this.camera.scale, 1.0 *
					 * this.camera.scale, 1.0 * this.camera.scale); g.setColor(new Color(1.0f,
					 * 0.30f, 0.30f, (float) (0.3 + ((temperature.getMatrix()[i][j] - 4.0) / 2))));
					 * g.fill(temp); g.draw(temp); } if (temperature.getMatrix()[i][j] >= 5 &&
					 * temperature.getMatrix()[i][j] < 6) { Rectangle2D.Double temp = new
					 * Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale, (i -
					 * yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale, 1.0 *
					 * this.camera.scale); g.setColor(new Color(1.0f, 0.20f, 0.20f, (float) (0.3 +
					 * ((temperature.getMatrix()[i][j] - 5.0) / 2)))); g.fill(temp); g.draw(temp); }
					 * if (temperature.getMatrix()[i][j] >= 6 && temperature.getMatrix()[i][j] < 7)
					 * { Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2)
					 * * this.camera.scale, (i - yWorldDimension / 2) * this.camera.scale, 1.0 *
					 * this.camera.scale, 1.0 * this.camera.scale); g.setColor(new Color(1.0f, 0.1f,
					 * 0.10f, (float) (0.3 + ((temperature.getMatrix()[i][j] - 6.0) / 2))));
					 * g.fill(temp); g.draw(temp); }
					 */
					if (temperature.getMatrix()[i][j] >= 1 && temperature.getMatrix()[i][j] < 4) {
						Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale,
								(i - yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale,
								1.0 * this.camera.scale);
						float alpha = (float) ((temperature.getMatrix()[i][j] - 1) / 4);
						g.setColor(new Color(1.0f, 1.0f, 0.10f, (float) ((alpha / 2))));
						g.fill(temp);
						g.draw(temp);
					}
					if (temperature.getMatrix()[i][j] >= 4 && temperature.getMatrix()[i][j] < 10) {
						Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale,
								(i - yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale,
								1.0 * this.camera.scale);
						float alpha = (float) ((temperature.getMatrix()[i][j] - 4) / 10);
						g.setColor(new Color(1.0f, 0.75f, 0.0f, (float) (0.3 + (alpha / 2))));
						g.fill(temp);
						g.draw(temp);
					}
					if (temperature.getMatrix()[i][j] >= 10 && temperature.getMatrix()[i][j] < 25) {
						Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale,
								(i - yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale,
								1.0 * this.camera.scale);
						float alpha = (float) ((temperature.getMatrix()[i][j] - 10) / 25);
						g.setColor(new Color(1.0f, 0.0f, 0.0f, (float) (0.3 + (alpha / 2))));
						g.fill(temp);
						g.draw(temp);
					}
					if (temperature.getMatrix()[i][j] >= 25 && temperature.getMatrix()[i][j] < 35) {
						Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale,
								(i - yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale,
								1.0 * this.camera.scale);
						float alpha = (float) ((temperature.getMatrix()[i][j] - 25) / 35);
						g.setColor(new Color(0.90f, 0.0f, 0.0f, (float) (0.5 + (alpha / 2))));
						g.fill(temp);
						g.draw(temp);
					}
					if (temperature.getMatrix()[i][j] >= 35 && temperature.getMatrix()[i][j] < 60) {
						Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale,
								(i - yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale,
								1.0 * this.camera.scale);
						float alpha = (float) ((temperature.getMatrix()[i][j] - 35) / 50);
						g.setColor(new Color(0.40f, 0.0f, 0.0f, (float) (0.5 + (alpha / 2))));
						g.fill(temp);
						g.draw(temp);
					}
					/*
					 * if (temperature.getMatrix()[i][j] >= 50 && temperature.getMatrix()[i][j] <
					 * 60) { Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension /
					 * 2) * this.camera.scale, (i - yWorldDimension / 2) * this.camera.scale, 1.0 *
					 * this.camera.scale, 1.0 * this.camera.scale); g.setColor(new Color(0.40f,
					 * 0.0f, 0.0f, 0.8f)); g.fill(temp); g.draw(temp); }
					 */
					if (temperature.getMatrix()[i][j] >= 60 && temperature.getMatrix()[i][j] < 70) {
						Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale,
								(i - yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale,
								1.0 * this.camera.scale);
						g.setColor(new Color(0.35f, 0.0f, 0.0f, 0.9f));
						g.fill(temp);
						g.draw(temp);
					}
					if (temperature.getMatrix()[i][j] >= 70) {
						Rectangle2D.Double temp = new Rectangle2D.Double((j - xWorldDimension / 2) * this.camera.scale,
								(i - yWorldDimension / 2) * this.camera.scale, 1.0 * this.camera.scale,
								1.0 * this.camera.scale);
						g.setColor(new Color(0.25f, 0.0f, 0.0f, 0.9f));
						g.fill(temp);
						g.draw(temp);
					}
				}
			}
		}

		if (this.renderLidar.isActive()) {
			for (Ray ray : rayList) {
				double xEnd = ray.getStart().x + Lidar.rayLength * Math.cos(ray.getDirection());
				double yEnd = ray.getStart().y + Lidar.rayLength * Math.sin(ray.getDirection());
				// System.out.println(ray.getDirectionVector());
				double xStart = ray.getStart().x;
				double yStart = ray.getStart().y;
				// System.out.println(ray.getStart());

				Line2D.Double line = new Line2D.Double(xStart * this.camera.scale, yStart * this.camera.scale,
						xEnd * this.camera.scale, yEnd * this.camera.scale);
				g.setColor(Color.RED);
				g.draw(line);
			}
			rayList.clear();
		}

		if (this.renderSmellRadius.isActive()) {
			for (Worm worm : Prova.worms) {
				double direction = worm.getBodies().get(0).getWorldCenter()
						.subtract(worm.getBodies().get(1).getWorldCenter()).getDirection();
				Vector2 vec = new Vector2(direction);
				// Vector2 norm = directionVector.getNormalized().multiply(Nose.smellRadius);
				Line2D.Double line1 = new Line2D.Double(worm.getBodies().get(0).getWorldCenter().x * this.camera.scale,
						worm.getBodies().get(0).getWorldCenter().y * this.camera.scale,
						(worm.getBodies().get(0).getWorldCenter().x + Nose.smellRadius * Math.cos(direction))
								* this.camera.scale,
						(worm.getBodies().get(0).getWorldCenter().y + Nose.smellRadius * Math.sin(direction))
								* this.camera.scale);
				Line2D.Double line2 = new Line2D.Double(worm.getBodies().get(0).getWorldCenter().x * this.camera.scale,
						worm.getBodies().get(0).getWorldCenter().y * this.camera.scale,
						(worm.getBodies().get(0).getWorldCenter().x - Nose.smellRadius * Math.cos(direction))
								* this.camera.scale,
						(worm.getBodies().get(0).getWorldCenter().y - Nose.smellRadius * Math.sin(direction))
								* this.camera.scale);
				Ellipse2D.Double e = new Ellipse2D.Double(
						(worm.getBodies().get(0).getWorldCenter().x - Nose.smellRadius) * this.camera.scale,
						(worm.getBodies().get(0).getWorldCenter().y - Nose.smellRadius) * this.camera.scale,
						Nose.smellRadius * 2 * this.camera.scale, Nose.smellRadius * 2 * this.camera.scale);
				g.setColor(new Color(0.26f, 0.38f, 0.47f, 0.4f));
				g.fill(e);
				g.draw(e);
				g.setColor(Color.MAGENTA);
				g.draw(line1);
				g.draw(line2);

			}
		}

		// draw all the objects in the world
		for (int i = 0; i < this.world.getBodyCount(); i++) {
			// get the object
			SimulationBody body = (SimulationBody) this.world.getBody(i);
			this.render(g, elapsedTime, body);

			// body aabb
			if (this.renderBodyAABBs.isActive()) {
				AABB aabb = this.world.getContinuousCollisionDetectionBroadphaseDetector().getAABB(body);
				Rectangle2D.Double ce = new Rectangle2D.Double(aabb.getMinX() * this.camera.scale,
						aabb.getMinY() * this.camera.scale, aabb.getWidth() * this.camera.scale,
						aabb.getHeight() * this.camera.scale);
				g.setColor(Color.CYAN);
				g.draw(ce);
			}

			// body rotation radius
			if (this.renderBodyRotationRadius.isActive()) {
				Vector2 c = body.getWorldCenter();
				double r = body.getRotationDiscRadius();
				Ellipse2D.Double e = new Ellipse2D.Double((c.x - r) * this.camera.scale, (c.y - r) * this.camera.scale,
						r * 2 * this.camera.scale, r * 2 * this.camera.scale);
				g.setColor(Color.PINK);
				g.draw(e);
			}

			// loop over all the body fixtures for this body
			for (BodyFixture fixture : body.getFixtures()) {
				// fixture AABB
				if (this.renderFixtureAABBs.isActive()) {
					AABB aabb = this.world.getBroadphaseDetector().getAABB(body, fixture);
					Rectangle2D.Double ce = new Rectangle2D.Double(aabb.getMinX() * this.camera.scale,
							aabb.getMinY() * this.camera.scale, aabb.getWidth() * this.camera.scale,
							aabb.getHeight() * this.camera.scale);
					g.setColor(Color.MAGENTA);
					g.draw(ce);
				}

				// fixture radius
				if (this.renderFixtureRotationRadius.isActive()) {
					Transform tx = body.getTransform();
					Vector2 c = tx.getTransformed(fixture.getShape().getCenter());
					double r = fixture.getShape().getRadius();
					Ellipse2D.Double e = new Ellipse2D.Double((c.x - r) * this.camera.scale,
							(c.y - r) * this.camera.scale, r * 2 * this.camera.scale, r * 2 * this.camera.scale);
					g.setColor(Color.CYAN.darker());
					g.draw(e);
				}
			}
		}

		for (int i = 0; i < this.world.getJointCount(); i++) {
			Joint<SimulationBody> j = this.world.getJoint(i);
			if (j instanceof DistanceJoint) {
				DistanceJoint<SimulationBody> dj = (DistanceJoint<SimulationBody>) j;
				Line2D.Double vn = new Line2D.Double(dj.getAnchor1().x * this.camera.scale,
						dj.getAnchor1().y * this.camera.scale, dj.getAnchor2().x * this.camera.scale,
						dj.getAnchor2().y * this.camera.scale);
				double target = dj.getDistance();
				double val = Math.abs(target - dj.getAnchor1().distance(dj.getAnchor2())) * 100;
				int red = (int) Math.floor(Math.min(val, 255));
				g.setColor(new Color(red, 0, 0));
				g.draw(vn);
			} else if (j instanceof PinJoint) {
				PinJoint<SimulationBody> pj = (PinJoint<SimulationBody>) j;
				Line2D.Double vn = new Line2D.Double(pj.getAnchor1().x * this.camera.scale,
						pj.getAnchor1().y * this.camera.scale, pj.getAnchor2().x * this.camera.scale,
						pj.getAnchor2().y * this.camera.scale);
				double max = pj.getMaximumForce();
				double val = pj.getReactionForce(this.world.getTimeStep().getInverseDeltaTime()).getMagnitude();
				int red = (int) Math.floor((val / max) * 255);
				g.setColor(new Color(red, 0, 0));
			
				g.draw(vn);
			}
		}

		if (this.renderContacts.isActive()) {
			this.drawContacts(g);
		}

		if (this.renderSensorValues.isActive()) {
			int yScale = 40;
			double screenHeight = this.canvas.getHeight();
			double screenWidth = this.canvas.getWidth();
			double boxHeight = 0;
			for (Worm worm : Prova.worms) {
				boxHeight += 30;
			}
			Rectangle2D.Double rect = new Rectangle2D.Double(-screenWidth / 2, -screenHeight / 2, 1240, 20 + boxHeight);
			AffineTransform yFlip = AffineTransform.getScaleInstance(1, -1);
			g.transform(yFlip);
			g.setColor(new Color(1.00f, 1.00f, 1.00f, 0.5f));
			g.fill(rect);
			g.draw(rect);
			for (Worm worm : Prova.worms) {
				try {

					Rectangle2D.Double rectColor = new Rectangle2D.Double(-screenWidth / 2 + 8,
							-screenHeight / 2 + yScale - 9, 10, 10);
					g.setColor(worm.getGenoColor());
					g.fill(rectColor);
					g.draw(rectColor);
					g.setColor(Color.black);
					// g.drawString("Worm " + Prova.worms.indexOf(worm),
					// Math.round((worm.getBodies().get(0).getWorldCenter().x)* this.camera.scale),
					// (int) Math.round((worm.getBodies().get(0).getWorldCenter().y+5)*
					// this.camera.scale));

					g.drawString("Worm " + Prova.worms.indexOf(worm), (int) Math.round(-screenWidth / 2 + 25),
							(int) Math.round(-screenHeight / 2 + yScale));
					g.drawString("Current energy: " + worm.getCurrentEnergy(), (int) Math.round(-screenWidth / 2 + 80),
							(int) Math.round(-screenHeight / 2 + yScale));
					g.drawString("Food items (left): " + worm.getCurrentFoodItems()[0],
							(int) Math.round(-screenWidth / 2 + 210), (int) Math.round(-screenHeight / 2 + yScale));
					g.drawString("Food items (right): " + worm.getCurrentFoodItems()[1],
							(int) Math.round(-screenWidth / 2 + 320), (int) Math.round(-screenHeight / 2 + yScale));
					BigDecimal bd = new BigDecimal(worm.getCurrentClosestItemDistance()).setScale(2,
							RoundingMode.HALF_UP);

					double touchRight = worm.getCurrentTouchRight();
					if (touchRight == 1d) {
						g.drawString("Touch sensor(right): food", (int) Math.round(-screenWidth / 2 + 657),
								(int) Math.round(-screenHeight / 2 + yScale));
					} else if (touchRight == -1d) {
						g.drawString("Touch sensor(right): other", (int) Math.round(-screenWidth / 2 + 657),
								(int) Math.round(-screenHeight / 2 + yScale));
					} else if (touchRight == 2d) {
						g.drawString("Touch sensor(right): self", (int) Math.round(-screenWidth / 2 + 657),
								(int) Math.round(-screenHeight / 2 + yScale));
					} else if (touchRight == 0d) {
						g.drawString("Touch sensor(right): nothing", (int) Math.round(-screenWidth / 2 + 657),
								(int) Math.round(-screenHeight / 2 + yScale));
					}

					double touchLeft = worm.getCurrentTouchLeft();
					if (touchLeft == 1d) {
						g.drawString("Touch sensor(left): food", (int) Math.round(-screenWidth / 2 + 820),
								(int) Math.round(-screenHeight / 2 + yScale));
					} else if (touchLeft == -1d) {
						g.drawString("Touch sensor(left): other", (int) Math.round(-screenWidth / 2 + 820),
								(int) Math.round(-screenHeight / 2 + yScale));
					} else if (touchLeft == 2d) {
						g.drawString("Touch sensor(left): self", (int) Math.round(-screenWidth / 2 + 820),
								(int) Math.round(-screenHeight / 2 + yScale));
					} else if (touchLeft == 0d) {
						g.drawString("Touch sensor(left): nothing", (int) Math.round(-screenWidth / 2 + 820),
								(int) Math.round(-screenHeight / 2 + yScale));
					}
					g.drawString("Gen: " + worm.getGeneration(), (int) Math.round(-screenWidth / 2 + 972),
							(int) Math.round(-screenHeight / 2 + yScale));
					g.drawString("Norm distance from closest item: " + bd.doubleValue(),
							(int) Math.round(-screenWidth / 2 + 440), (int) Math.round(-screenHeight / 2 + yScale));

					g.drawString(
							"Avg temperature: "
									+ temperature.getMeanTemperature(worm.getBodies().get(0).getWorldCenter().x,
											worm.getBodies().get(0).getWorldCenter().y),
							(int) Math.round(-screenWidth / 2 + 1015), (int) Math.round(-screenHeight / 2 + yScale));
					yScale += 25;
				} catch (NullPointerException e) {
					System.err.println("Exception " + e);
				}
			}
		}

	}

	private void drawContacts(Graphics2D g) {
		Iterator<WorldCollisionData<SimulationBody>> it = this.world.getCollisionDataIterator();
		while (it.hasNext()) {
			WorldCollisionData<SimulationBody> wcd = it.next();

			if (!wcd.isContactConstraintCollision())
				continue;

			ContactConstraint<SimulationBody> cc = wcd.getContactConstraint();
			for (SolvedContact c : cc.getContacts()) {
				// draw the contact point
				final double r = 2.5 / this.camera.scale;
				final double d = r * 2;
				Ellipse2D.Double cp = new Ellipse2D.Double((c.getPoint().x - r) * this.camera.scale,
						(c.getPoint().y - r) * this.camera.scale, d * this.camera.scale, d * this.camera.scale);
				g.setColor(Color.ORANGE);
				g.fill(cp);

				// check for sensor/enabled
				if (!cc.isSensor() && cc.isEnabled()) {
					// draw the contact normal
					Line2D.Double vn = new Line2D.Double(c.getPoint().x * this.camera.scale,
							c.getPoint().y * this.camera.scale,
							(c.getPoint().x - cc.getNormal().x * c.getDepth() * 100) * this.camera.scale,
							(c.getPoint().y - cc.getNormal().y * c.getDepth() * 100) * this.camera.scale);
					g.setColor(Color.BLUE);
					g.draw(vn);
				}
			}
		}
	}

	/**
	 * Renders the body.
	 * 
	 * @param g           the graphics object to render to
	 * @param elapsedTime the elapsed time from the last update
	 * @param body        the body to render
	 */
	protected void render(Graphics2D g, double elapsedTime, SimulationBody body) {
		// if the object is selected, draw it magenta
		Color color = body.getColor();
		if (this.picking.isEnabled() && this.picking.isActive() && this.picking.getBody() == body) {
			color = Color.MAGENTA;
		}

		// draw the object
		body.render(g, this.camera.scale, color);
	}

	protected Vector2 toWorldCoordinates(Point p) {
		return this.camera.toWorldCoordinates(this.canvas.getWidth(), this.canvas.getHeight(), p);
	}

	/**
	 * Used to handle any input events or custom code.
	 */
	protected void handleEvents() {

	}

	/**
	 * Stops the simulation.
	 */
	public synchronized void stop() {
		this.stopped = true;
	}

	/**
	 * Returns true if the simulation is stopped.
	 * 
	 * @return boolean true if stopped
	 */
	public boolean isStopped() {
		return this.stopped;
	}

	/**
	 * Pauses the simulation.
	 */
	public void pause() {
		this.paused.setActive(true);
	}

	/**
	 * Pauses the simulation.
	 */
	public synchronized void resume() {
		this.last = System.nanoTime();
		this.paused.setActive(false);
	}

	/**
	 * Returns true if the simulation is paused.
	 * 
	 * @return boolean true if paused
	 */
	public boolean isPaused() {
		return this.paused.isActive();
	}

	/**
	 * Returns true if mouse picking is enabled.
	 * 
	 * @return boolean
	 */
	public boolean isMousePickingEnabled() {
		return this.picking.isEnabled();
	}

	/**
	 * Sets mouse picking enabled.
	 * 
	 * @param flag true if mouse picking should be enabled
	 */
	public void setMousePickingEnabled(boolean flag) {
		this.picking.setEnabled(flag);
	}

	/**
	 * Returns true if mouse panning is enabled.
	 * 
	 * @return boolean
	 */
	public boolean isMousePanningEnabled() {
		return this.panning.isEnabled();
	}

	/**
	 * Sets mouse panning enabled.
	 * 
	 * @param flag true if mouse panning should be enabled
	 */
	public void setMousePanningEnabled(boolean flag) {
		this.panning.setEnabled(flag);
	}

	/**
	 * Returns true if fixture AABB drawing is enabled.
	 * 
	 * @return boolean
	 */
	public boolean isFixtureAABBDrawingEnabled() {
		return this.renderFixtureAABBs.isActive();
	}

	/**
	 * Sets whether fixture AABB drawing is enabled.
	 * 
	 * @param flag true if drawing should be enabled
	 */
	public void setFixtureAABBDrawingEnabled(boolean flag) {
		this.renderFixtureAABBs.setActive(flag);
	}

	/**
	 * Returns true if body AABB drawing is enabled.
	 * 
	 * @return boolean
	 */
	public boolean isBodyAABBDrawingEnabled() {
		return this.renderBodyAABBs.isActive();
	}

	/**
	 * Sets whether body AABB drawing is enabled.
	 * 
	 * @param flag true if drawing should be enabled
	 */
	public void setBodyAABBDrawingEnabled(boolean flag) {
		this.renderBodyAABBs.setActive(flag);
	}

	/**
	 * Returns true if fixture rotation radius drawing is enabled.
	 * 
	 * @return boolean
	 */
	public boolean isFixtureRotationRadiusDrawingEnabled() {
		return this.renderFixtureRotationRadius.isActive();
	}

	/**
	 * Sets whether fixture rotation radius drawing is enabled.
	 * 
	 * @param flag true if drawing should be enabled
	 */
	public void setFixtureRotationRadiusDrawingEnabled(boolean flag) {
		this.renderFixtureRotationRadius.setActive(flag);
	}

	/**
	 * Returns true if body rotation radius drawing is enabled.
	 * 
	 * @return boolean
	 */
	public boolean isBodyRotationRadiusDrawingEnabled() {
		return this.renderBodyRotationRadius.isActive();
	}

	/**
	 * Sets whether body rotation radius drawing is enabled.
	 * 
	 * @param flag true if drawing should be enabled
	 */
	public void setBodyRotationRadiusDrawingEnabled(boolean flag) {
		this.renderBodyRotationRadius.setActive(flag);
	}

	/**
	 * Returns true if contact drawing is enabled.
	 * 
	 * @return boolean
	 */
	public boolean isContactDrawingEnabled() {
		return this.renderContacts.isActive();
	}

	/**
	 * Sets if contact drawing is enabled.
	 * 
	 * @param flag true if contact drawing should be enabled
	 */
	public void setContactDrawingEnabled(boolean flag) {
		this.renderContacts.setActive(flag);
	}

	/**
	 * Returns the current scale (x pixels / meter)
	 * 
	 * @return double
	 */
	public double getScale() {
		return this.camera.scale;
	}

	/**
	 * Sets the scale (zoom).
	 * 
	 * @param scale the number of pixels per meter
	 */
	public void setScale(double scale) {
		this.camera.scale = scale;
	}

	/**
	 * Returns the x offset (pan-x).
	 * 
	 * @return double
	 */
	public double getOffsetX() {
		return this.camera.offsetX;
	}

	/**
	 * Sets the x offset (pan-x).
	 * 
	 * @param offsetX the x offset in pixels
	 */
	public void setOffsetX(double offsetX) {
		this.camera.offsetX = offsetX;
	}

	/**
	 * Returns the y offset (pan-y).
	 * 
	 * @return double
	 */
	public double getOffsetY() {
		return this.camera.offsetY;
	}

	/**
	 * Sets the y offset (pan-y).
	 * 
	 * @param offsetY the y offset in pixels
	 */
	public void setOffsetY(double offsetY) {
		this.camera.offsetY = offsetY;
	}

	/**
	 * Generates Java code for the current state of the world.
	 * 
	 * @return String
	 */
	public String toCode() {
		return CodeExporter.export(this.getName(), this.world);
	}

	/**
	 * Starts the simulation.
	 */
	public void run() {
		// set the look and feel to the system look and feel
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		} catch (InstantiationException e) {
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		} catch (UnsupportedLookAndFeelException e) {
			e.printStackTrace();
		}

		// show it
		this.setVisible(true);

		// start it
		this.start();
	}
}
