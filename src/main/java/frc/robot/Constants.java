// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
		public static final double podRotationUpperBound = 0.4;
		public static final double podRotationLowerBound = -0.4; // TODO: Double check with CAD
		public static final double driveMetersPerMotorRotation = Units.inchesToMeters(2) * Math.PI / 1 / 0.36; // Wheel
																												// Diameter
																												// M *
																												// PI /
																												// Enc
																												// Count
																												// Per
																												// Rev /
																												// Gear
																												// Ratio

		public static final int pigeonID = 25;

		public static final class PodConfig {
			public final int leftMotorID;
			public final int rightMotorID;
			public final int encoderID;
			public final double encoderOffset;
			public PodConfig(int leftMotorID, int rightMotorID, int encoderID, double encoderOffset) {
				this.leftMotorID = leftMotorID;
				this.rightMotorID = rightMotorID;
				this.encoderID = encoderID;
				this.encoderOffset = encoderOffset;
			}
			public static final boolean encoderInvert = false;
			public static final boolean leftMotorInvert = false; 
			public static final boolean rightMotorInvert = false; 

			public static final double motorGearing = 50;

			public static final boolean motorsBrake = true;
			public static final int ampLimit = 80;
			public static final double maxOutput = 1;
			public static final double rampRate = 0.2;

			public static final double azimuthkP = 1.2;
			public static final double azimuthkI = 0.02;
			public static final double azimuthkD = 0.001;
			public static final double azimuthkS = 0.0;
		}
		public static final PodConfig[] PodConfigs = {
			new PodConfig(0, 1, 0, 0d),
			new PodConfig(3, 4, 1, 0d),
			new PodConfig(6, 7, 2, 0d),
			new PodConfig(9, 10, 3, 0d)
		};
	}

	public final class RobotConfig {
		public static final PIDController gyroPID = new PIDController(0.046, 0d, 0.001);

		//TODO: get actual numbers for these
		private static final double wheelBase = 5.75;
		private static final Translation2d frontLeftPodPosition = new Translation2d(Units.inchesToMeters(wheelBase),
				Units.inchesToMeters(wheelBase));
		private static final Translation2d frontRightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase),
				-Units.inchesToMeters(wheelBase));
		private static final Translation2d backLeftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase),
				Units.inchesToMeters(wheelBase));
		private static final Translation2d backRightPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase),
				-Units.inchesToMeters(wheelBase));
		public static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(
				frontLeftPodPosition, frontRightPodPosition, backLeftPodPosition, backRightPodPosition);
		

		public static final double robotMaxSpeed = 3.99; // meters per second

	}

	public final class SimConstants {
		public static final double inertia = 31.605; // kg*m^2
		public static final double mass = 226; // kg, approximate mass of the robot
		public static final double wheelRadius = 0.3; // meters
		public static final double trackWidth = 0.5; // meters, distance between left and right wheels
		public static final double gearRatio = PodConfig.motorGearing; // gear ratio of the drivetrain

	}
}
