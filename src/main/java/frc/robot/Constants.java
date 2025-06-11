// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
	public static final int driverPort = 0;

	/** CAN ID, Invert, Pod Positions, Offsets, Conversion Rates */
	public final class RobotMap {
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

		public final class pods {
			public final int leftMotorID;
			public final int rightMotorID;
			public final int encoderID;
			public final double encoderOffset;
			public pods(int leftMotorID, int rightMotorID, int encoderID, double encoderOffset) {
				this.leftMotorID = leftMotorID;
				this.rightMotorID = rightMotorID;
				this.encoderID = encoderID;
				this.encoderOffset = encoderOffset;
			}
			public static final boolean azimuthInvert = false;
			public static final boolean leftMotorInvert = false; 
			public static final boolean rightMotorInvert = false; 
			public static final boolean encoderInvert = false;

			public static final double azimuthRadiansPerRotation = 1;

			public static final boolean motorsBrake = true;
			public static final int ampLimit = 80;
			public static final double maxOutput = 1;
			public static final double rampRate = 0.2;

			public static final double azimuthkP = 1.2;
			public static final double azimuthkI = 0.02;
			public static final double azimuthkD = 0.001;
			public static final double azimuthkS = 0.0;
		}
		public static final pods[] Pods = new pods[4];
		{
			Pods[0] = new pods(0, 1, 2, 0d);
			Pods[1] = new pods(3, 4, 5, 0d);
			Pods[2] = new pods(6, 7, 8, 0d);
			Pods[3] = new pods(9, 10, 11, 0d);
		}
	}

	public final class RobotConfig {
		public static final PIDController gyroPID = new PIDController(0.046, 0d, 0.001);

		private static final double wheelBase = 5.75;
		private static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase),
				Units.inchesToMeters(wheelBase));
		private static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase),
				-Units.inchesToMeters(wheelBase));
		public static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(leftPodPosition,
				rightPodPosition);

		public static final double robotMaxSpeed = 3.99; // meters per second

	}
}
