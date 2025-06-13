// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotMap.PodConfig;

/** Add your docs here. */
public final class Constants {
	public static final int driverPort = 0;

	/** CAN ID, Invert, Pod Positions, Offsets, Conversion Rates */
	public final class RobotMap {
		public static final double podRotationUpperBound = 210.0/360.0*0.5;
		public static final double podRotationLowerBound = -210.0/360.0*0.5;
		public static final double driveMetersPerMotorRotation = Units.inchesToMeters(2) * Math.PI / 1 / 0.36; 

		public static final int pigeonID = 25;

		// Camera IDs. this is for individual camera-threads, but there's only one so its fine
		public static enum CameraName {
			front
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
			public static final boolean encoderInvert = false;
			public static final boolean leftMotorInvert = true; 
			public static final boolean rightMotorInvert = false; 

			public static final double motorGearing = 50;

			public static final boolean motorsBrake = true;
			public static final int ampLimit = 80;
			public static final double maxOutput = 1;
			public static final double rampRate = 0.2;

			public static final double azimuthkP = 0.6;
			public static final double azimuthkI = 0.0;
			public static final double azimuthkD = 0.0;
			public static final double azimuthkS = 0.0;
		}
		private static final double wheelBase = 1.0;
		private static final double trackWidth = 1.0;
		public static final PodConfig[] PodConfigs = {
			new PodConfig(0, 1, 0, 0d, new Translation2d(wheelBase / 2, trackWidth / 2)),
			new PodConfig(3, 4, 1, 0d, new Translation2d(wheelBase / 2, -trackWidth / 2)),
			new PodConfig(6, 7, 2, 0d, new Translation2d(-wheelBase / 2, trackWidth / 2)),
			new PodConfig(9, 10, 3, 0d, new Translation2d(-wheelBase / 2, -trackWidth / 2))
		};
		public static SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(
			java.util.Arrays.stream(PodConfigs)
				.map(pod -> pod.position)
				.toArray(Translation2d[]::new)
		);
	}

	public final class RobotConfig {
		public static final PIDController gyroPID = new PIDController(0.046, 0d, 0.001);

		public static final double robotMaxSpeed = 3.99; // meters per second
		public static final double motorMaxOutput = 1.0; // max output of the motors

	}

	public final class SimConstants {
		public static final double inertia = 0.0605; // kg*m^2
		public static final double mass = 50; // kg, approximate mass of the robot
		public static final double wheelRadius = 0.3; // meters
		public static final double trackWidth = 0.5; // meters, distance between left and right wheels
		public static final double gearRatio = 1/PodConfig.motorGearing; // gear ratio of the drivetrain

	}
}
