// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class TurdConstants {
	public static boolean tuningMode = false;
	public class RobotConfig {
		public static final double driveMetersPerMotorRotation = 1/(Units.inchesToMeters(2) * Math.PI / 1.36); //Wheel Diameter M * PI / Enc Count Per Rev / Gear Ratio

        public static final int pigeonID = 25;
        public static final PIDController gyroPID = new PIDController(0.046, 0d, 0.001);

        public static final class PodConfig {
			public final int azimuthID;
			public final int driveID;
			public final int encoderID;
			public final double encoderOffset;
			public final Translation2d position;
			public PodConfig(int azimuthID, int driveID, int encoderID, double encoderOffset, Translation2d podPosition) {
				this.azimuthID = azimuthID;
				this.driveID = driveID;
				this.encoderID = encoderID;
				this.encoderOffset = encoderOffset;
				this.position = podPosition;
			}
			public static final boolean encoderInvert = true;
			public static final boolean azimuthInvert = false; 
			public static final boolean driveInvert = true; 

			public static final double motorGearing = 50;

			public static final boolean motorsBrake = true;
			public static final int ampLimit = 80;
			public static final double maxOutput = 1;
			public static final double rampRate = 0.2;

			public static final double kP = 1;
			public static final double kI = 0.0;
			public static final double kD = 0.2;
		}

		private static final double wheelBase = Units.inchesToMeters(11.054);
		private static final double trackWidth = Units.inchesToMeters(11.054);
		public static final Translation2d offsetPosition = new Translation2d(0.0, Constants.IS_MASTER ? 0.75 : -0.75); // center of rotation of robot

		public static final class SingleRobotConfig {
			public PodConfig[] PodConfigs;
			public SwerveDriveKinematics drivetrainKinematics;
			public SingleRobotConfig(PodConfig[] podConfigs) {
				this.PodConfigs = podConfigs;
				this.drivetrainKinematics = new SwerveDriveKinematics(
					java.util.Arrays.stream(podConfigs)
						.map(pod -> pod.position)
						.toArray(Translation2d[]::new)
				);
			}
		}
		public static final SingleRobotConfig[] robotConfigs = new SingleRobotConfig[] { 
		new SingleRobotConfig(new PodConfig[] { // prowl AKA master AKA og megatron
			new PodConfig(16, 15, 23, -0.8865d, new Translation2d(wheelBase / 2, trackWidth / 2)),
			new PodConfig(14, 13, 22, 0.3328d, new Translation2d(wheelBase / 2, -trackWidth / 2)),
			new PodConfig(18, 17, 24, 0.8132d, new Translation2d(-wheelBase / 2, trackWidth / 2)),
			new PodConfig(12, 11, 21, -0.4138d + (1d/33d), new Translation2d(-wheelBase / 2, -trackWidth / 2))
		}),
		new SingleRobotConfig(new PodConfig[] { // NemesisPrime AKA slave AKA Kris's NERDSwerve
			new PodConfig(16, 15, 23, 0.9844, new Translation2d(wheelBase / 2, trackWidth / 2)),
			new PodConfig(14, 13, 22, 0.4797, new Translation2d(wheelBase / 2, -trackWidth / 2)),
			new PodConfig(18, 17, 24, -0.6980, new Translation2d(-wheelBase / 2, trackWidth / 2)),
			new PodConfig(12, 11, 21, -0.4006, new Translation2d(-wheelBase / 2, -trackWidth / 2))
		})};


        public static final double robotMaxSpeed = 3.99; //meters per second


        // Azimuth Settings
        public static final boolean azimuthBrake = true;

        public static final int azimuthAmpLimit = 80;
        public static final double azimuthMaxOutput = 1;


        public static final double azimuthkP = 1.2;
        
        public static final double azimuthkI = 0.02;
        public static final double azimuthkD = 0.001;
        public static final double azimuthkS = 0.0;

        public static final double azimuthDriveSpeedMultiplier = 0.5;

        public static final double azimuthMotorRampRate = 0.0;

        // Drive Settings
        public static final double podMaxSpeed = 1;

        public static final boolean driveBrake = false;

        public static final int driveAmpLimit = 80;
        public static final int boostDriveLimit = 90;
        public static final double driveMotorRampRate = 0.2;

        public static final double azimuthRadiansPerMotorRotation = 2.200000047683716;

	}
    
}
