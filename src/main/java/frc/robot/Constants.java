// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants.RobotMap.PodConfig;

public final class Constants {
	public static final boolean tuningMode = true; // if true, the robot will use the dashboard to get values for PIDs

    //grab the master/slave from a file on the robot. This will determine whether it's running the shuffleboard server.
    public static final Path MASTER_PATH = Paths.get("/home/lvuser/master");
    public static final boolean IS_MASTER = MASTER_PATH.toFile().exists();
	//TODO: phase out if statements and replace them with switches w/ this enum
	public static enum RobotType {
		master, slave
	} 

	public static RobotType currentRobot = IS_MASTER ? RobotType.master : RobotType.slave;

	/*
	* CAN ID, Invert, Pod Positions, Offsets, Conversion Rates */
	public final class RobotMap {
		public static final double podRotationUpperBound = 210.0/360.0*0.5;
		public static final double podRotationLowerBound = -210.0/360.0*0.5;
		public static final double driveMetersPerMotorRotation = Units.inchesToMeters(2) * Math.PI / 1 / 0.36; 

		public static final int pigeonID = 25;

		private static final double wheelBase = 1.0;
		private static final double trackWidth = 1.0;
		public static final PodConfig[] PodConfigs = {
			new PodConfig(0, 1, 2, 0d, new Translation2d(wheelBase / 2, trackWidth / 2)),
			new PodConfig(3, 4, 5, 0d, new Translation2d(wheelBase / 2, -trackWidth / 2)),
			new PodConfig(6, 7, 8, 0d, new Translation2d(-wheelBase / 2, trackWidth / 2)),
			new PodConfig(9, 10, 11, 0d, new Translation2d(-wheelBase / 2, -trackWidth / 2))
		};
		public static SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(
			java.util.Arrays.stream(PodConfigs)
				.map(pod -> pod.position)
				.toArray(Translation2d[]::new)
		);



		// Camera IDs. this is for individual camera-threads, but there's only one so its fine
		public static enum CameraName {
			master, slave
		}

		public static final class PodConfig {
			public final int leftMotorID;
			public final int rightMotorID;
			public final int encoderID;
			public final double encoderOffset;
			public final Translation2d position;
			public PodConfig(int leftMotorID, int rightMotorID, int encoderID, double encoderOffset, Translation2d position) {
				this.leftMotorID = leftMotorID;
				this.rightMotorID = rightMotorID;
				this.encoderID = encoderID;
				this.encoderOffset = encoderOffset;
				this.position = position;
			}
			public static final boolean encoderInvert = true;
			public static final boolean leftMotorInvert = false; 
			public static final boolean rightMotorInvert = true; 

			public static final double motorGearing = 50;

			public static final boolean motorsBrake = true;
			public static final int ampLimit = 80;
			public static final double maxOutput = 1;
			public static final double rampRate = 0.2;

			public static final double kP = 1;
			public static final double kI = 0.0;
			public static final double kD = 0.2;
		}
	}

	public final class RobotConstants {
		public static final double robotMaxLinearSpeed = 1; // meters per second
		public static final double robotMaxRotationalSpeed = 2.0; // meters per second
		public static final double motorMaxOutput = 1.0; // max output of the motors

		//PID gains for tandem drive controller
		public static final double tandemkP = 0.3;
		public static final double tandemkI = 0.0;
		public static final double tandemkD = 0.0;

		public static final double tandemkP_angle = 0.3;
		public static final double tandemkI_angle = 0.0;
		public static final double tandemkD_angle = 0.0;

		public static final double tandemTranslation_deadband = 0.008;
		public static final double tandemAngle_deadband = 0.005;

		//forward, left, height; roll, pitch, yaw
		public static final Translation2d centerOfMasterToTag = new Translation2d(0, -0.2102); // meters, distance from the center of the master robot to the tag
		public static final Transform3d SLAVE_CAMERA_LOCATION = new Transform3d(new Translation3d(-0.152, -0.1996, 0.1350), new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(105)));
	}

	public final class JoystickConstants {
		public static final int driverPort = 0; // port of the driver controller

		public static final double deadband = 0.05; // deadband for the joysticks
		public static final double triggerDeadband = 0.05; // deadband for the triggers
	}

	public final class SimConstants {
		public static final double inertia = 0.0605; // kg*m^2
		public static final double mass = 50; // kg, approximate mass of the robot
		public static final double wheelRadius = 0.3; // meters
		public static final double trackWidth = 0.5; // meters, distance between left and right wheels
		public static final double gearRatio = 1/PodConfig.motorGearing; // gear ratio of the drivetrain

	}
}
