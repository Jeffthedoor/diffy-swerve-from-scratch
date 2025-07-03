// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants.RobotMap.PodConfig;

public final class Constants {
	public static final boolean tuningMode = true; // if true, the robot will use the dashboard to get values for PIDs

	/** CAN ID, Invert, Pod Positions, Offsets, Conversion Rates */
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
		public static final double robotMaxLinearSpeed = 3.99; // meters per second
		public static final double robotMaxRotationalSpeed = 3.0; // meters per second
		public static final double motorMaxOutput = 1.0; // max output of the motors

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0);

		// completely bs configs, taken directly from LIGHTING 862'S PREMIER ROBOT NAUTILUS FROM THE YEAR OF OUR LORD TWENTY TWENTY FIVE
		private static final double TRACK_WIDTH = Units.inchesToMeters(27);
        private static final Mass ROBOT_MASS = Pounds.of(147);
        private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(5.2268411);
        
		private static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
                Inches.of(2), MetersPerSecond.of(2), 
                1.916, DCMotor.getKrakenX60Foc(1).withReduction(6.75),
                Amps.of(120), 1);



        public static final RobotConfig CONFIG = new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG,
                new Translation2d[] { new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                        new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                        new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2) });
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
