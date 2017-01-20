package org.usfirst.frc.team1165.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the navX-MXP to implement a "rotate
 * to angle" feature.
 *
 * This example will automatically rotate the robot to one of four angles (0,
 * 90, 180 and 270 degrees).
 *
 * This rotation can occur when the robot is still, but can also occur when the
 * robot is driving. When using field-oriented control, this will cause the
 * robot to drive in a straight line, in whathever direction is selected.
 *
 * This example also includes a feature allowing the driver to "reset" the "yaw"
 * angle. When the reset occurs, the new gyro angle will be 0 degrees. This can
 * be useful in cases when the gyro drifts, which doesn't typically happen
 * during a FRC match, but can occur during long practice sessions.
 *
 * Note that the PID Controller coefficients defined below will need to be tuned
 * for your drive system.
 */

public class Robot extends SampleRobot implements PIDOutput
{
	AHRS ahrs;
	RobotDrive myRobot;
	Joystick stick;
	PIDController turnController;
	double rotateToAngleRate;

	/**
	 * This is a demo program showing the use of the navX-MXP to implement a
	 * collision detection feature, which can be used to detect events while
	 * driving a robot, such as bumping into a wall or being hit by another
	 * robot.
	 *
	 * The basic principle used within the Collision Detection example is the
	 * calculation of Jerk (which is defined as the change in acceleration). In
	 * the sample code shown below, both the X axis and the Y axis jerk are
	 * calculated, and if either exceeds a threshold, then a collision has
	 * occurred.
	 * 
	 * The collision threshold used in these samples will likely need to be
	 * tuned for your robot, since the amount of jerk which constitutes a
	 * collision will be dependent upon the robot mass and expected maximum
	 * velocity of either the robot, or any object which may strike the robot.
	 */
	double max_collision =0;

	double last_world_linear_accel_x;
	double last_world_linear_accel_y;
	int collisionDetected = 0;

	final static double kCollisionThreshold_DeltaG = 1.8f;

	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system. Note that the */
	/* SmartDashboard in Test mode has support for helping you tune */
	/* controllers by displaying a form where you can enter new P, I, */
	/* and D constants and test the mechanism. */

	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

	/* This tuning parameter indicates how close to "on target" the */
	/* PID Controller will attempt to get. */

	static final double kToleranceDegrees = 2.0f;

	CANTalon frontLeft = new CANTalon(0);
	CANTalon rearLeft = new CANTalon(1);
	CANTalon frontRight = new CANTalon(2);
	CANTalon rearRight = new CANTalon(3);

	public Robot()
	{
		myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
		myRobot.setExpiration(0.1);
		stick = new Joystick(0);

		frontLeft.setInverted(true);
		rearLeft.setInverted(true);
		try
		{
			/* Communicate w/navX-MXP via the MXP SPI Bus. */
			/*
			 * Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or
			 * SerialPort.Port.kUSB
			 */
			/*
			 * See
			 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/
			 * for details.
			 */
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex)
		{
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		/* Add the PID Controller to the Test-mode dashboard, allowing manual */
		/* tuning of the Turn Controller's P, I and D coefficients. */
		/* Typically, only the P value needs to be modified. */
		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	public void autonomous()
	{
		/*
		 * myRobot.setSafetyEnabled(false); myRobot.drive(0.0, 0.0); // stop
		 * robot Timer.delay(2.0); // for 2 seconds myRobot.drive(0.0, 0.0); //
		 * stop robot
		 */ }

	/**
	 * Runs the motors with onnidirectional drive steering.
	 * 
	 * Implements Field-centric drive control.
	 * 
	 * Also implements "rotate to angle", where the angle being rotated to is
	 * defined by one of four buttons.
	 * 
	 * Note that this "rotate to angle" approach can also be used while driving
	 * to implement "straight-line driving".
	 */
	public void operatorControl()
	{
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled())
		{
			double x = stick.getX() * stick.getX() * stick.getX();
			double y = stick.getY() * stick.getY() * stick.getY() * -1;
			double twist = stick.getTwist() * stick.getTwist() * stick.getTwist() * -1;
			if (Math.abs(x) < 0.1)
				x = 0;
			if (Math.abs(y) < 0.1)
				y = 0;
			if (Math.abs(twist) < 0.1)
				twist = 0;
			boolean rotateToAngle = false;
			// boolean evade = false;

			double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
			double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
			last_world_linear_accel_x = curr_world_linear_accel_x;
			double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
			double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
			last_world_linear_accel_y = curr_world_linear_accel_y;
			
			SmartDashboard.putNumber("Curr Accel Y", curr_world_linear_accel_y);
			SmartDashboard.putNumber("Last Accel Y", last_world_linear_accel_y);

			if (Math.abs(currentJerkY) > max_collision)
			{
				max_collision = Math.abs(currentJerkY);
			}
			
			SmartDashboard.putNumber("Max", max_collision);
			if ( /*
					 * ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
					 */
			Math.abs(currentJerkY) > kCollisionThreshold_DeltaG )
			{
				collisionDetected = 1;
				SmartDashboard.putString("Crash", "Bang");
			}
			SmartDashboard.putNumber("CollisionDetected", collisionDetected);
			if(Math.abs(currentJerkX) > 0.5 || Math.abs(currentJerkY) > 0.5)
			{
				SmartDashboard.putNumber("JerkX ", currentJerkX);
				SmartDashboard.putNumber("Jerk Y", currentJerkY);
			}

			if (stick.getRawButton(1))
			{
				ahrs.reset();
			}
			if (stick.getRawButton(2))
			{
				turnController.setSetpoint(0.0f);
				rotateToAngle = true;
			} else if (stick.getRawButton(3))
			{
				turnController.setSetpoint(90.0f);
				rotateToAngle = true;
			} else if (stick.getRawButton(4))
			{
				turnController.setSetpoint(179.9f);
				rotateToAngle = true;
			} else if (stick.getRawButton(5))
			{
				turnController.setSetpoint(-90.0f);
				rotateToAngle = true;
			} else if (stick.getRawButton(6) || collisionDetected == 1)
			{
				collisionDetected++;
				turnController.setSetpoint(-179.9);
				rotateToAngle = true;
				// evade = true;
				x = 0.75;
				y = 1;
			}
			double currentRotationRate;
			if (rotateToAngle)
			{
				turnController.enable();
				currentRotationRate = rotateToAngleRate;
				// if (evade) currentRotationRate = 1;
			} else
			{
				turnController.disable();
				currentRotationRate = twist;
				collisionDetected = 0;
			}
			try
			{
				// Use the joystick X axis for lateral movement,
				// Y axis for forward movement, and the current
				// calculated rotation rate (or joystick Z axis),
				// depending upon whether "rotate to angle" is active.
				myRobot.mecanumDrive_Cartesian(x, y, currentRotationRate, ahrs.getAngle());
			} catch (RuntimeException ex)
			{
				DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
			}
			Timer.delay(0.005); // wait for a motor update time
		}
	}

	/**
	 * Runs during test mode
	 */
	public void test()
	{
	}

	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX-MXP yaw angle input and PID Coefficients. */
	public void pidWrite(double output)
	{
		rotateToAngleRate = output;
	}
}