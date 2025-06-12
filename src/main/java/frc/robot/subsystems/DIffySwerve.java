// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMap.PodConfig;
import frc.robot.Constants.RobotConfig;

public class DIffySwerve extends SubsystemBase {
	private final Pigeon2 gyro;

	private final DrivePod[] pods = new DrivePod[4];

	private final SwerveDriveOdometry odometer;

	private ShuffleboardTab tab = Shuffleboard.getTab("PID");
	private GenericEntry azimuthP;
	private GenericEntry azimuthI;
	private GenericEntry azimuthD;
	private GenericEntry azimuthkS;
	private GenericEntry ADMult;

	private PIDController gyroPID;
	public double targetAngle = 0;
	private double odoAngleOffset = Math.PI * 0.0;

	private Rotation2d gyroResetAngle = new Rotation2d();

	private SwerveDriveKinematics drivetrainKinematics;
	private final double robotMaxSpeed;

	private final Field2d field2d = new Field2d();

	public DIffySwerve() {

		this.gyroPID = RobotConfig.gyroPID;
		this.drivetrainKinematics = RobotConfig.drivetrainKinematics;
		this.robotMaxSpeed = RobotConfig.robotMaxSpeed;
		gyroPID.enableContinuousInput(0.0, 2 * Math.PI);

		gyro = new Pigeon2(RobotMap.pigeonID);
		gyroPID.enableContinuousInput(0.0, 2 * Math.PI);


		for (int i = 0; i < pods.length; i++) {
			pods[i] = new DrivePod(RobotMap.PodConfigs[i].encoderID, RobotMap.PodConfigs[i].leftMotorID, RobotMap.PodConfigs[i].rightMotorID, PodConfig.leftMotorInvert, PodConfig.rightMotorInvert, RobotMap.PodConfigs[i].encoderOffset, PodConfig.encoderInvert, PodConfig.ampLimit, PodConfig.motorsBrake, PodConfig.rampRate, PodConfig.azimuthkP, PodConfig.azimuthkI, PodConfig.azimuthkD, PodConfig.maxOutput);
		}
		
		SwerveModulePosition positions[] = new SwerveModulePosition[pods.length];
		for (int i = 0; i < pods.length; i++) {
			positions[i] = pods[i].getPodPosition();
		}

		odometer = new SwerveDriveOdometry(drivetrainKinematics, new Rotation2d(0), positions);

		// gyro.configAllSettings(new Pigeon2Configuration());
	}

	public void resetOdometry(Pose2d pose) {
		odoAngleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI * 0.5 : Math.PI * 1.5;
		odometer.resetPosition(new Rotation2d(odoAngleOffset), getModulePositions(), pose);
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition positions[] = new SwerveModulePosition[pods.length];
		for (int i = 0; i < pods.length; i++) {
			positions[i] = pods[i].getPodPosition();
		}		return positions;
	}

	public void resetPods() {
		resetGyro();
		for (DrivePod pod : pods) {
			pod.resetPod();
		}

		//TODO: wtf hardcoded??
		resetOdometry(new Pose2d(new Translation2d(8.0, 4.2), new Rotation2d()));
	}

	public void resetZero() {
		for (DrivePod pod : pods) {
			pod.resetZero();
		}	
	}

	public void stop() {
		for (DrivePod pod : pods) {
			pod.stop();
		}
	}

	public Rotation2d getGyro() {
		return new Rotation2d(-gyro.getAngle() * Math.PI / 180).minus(gyroResetAngle);
	}

	public void resetGyro() {
		gyroResetAngle = getGyro().plus(gyroResetAngle);
		targetAngle = 0;
	}

	public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
		boolean manualTurn = true; // Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.1;

		chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
				chassisSpeeds.vyMetersPerSecond,
				manualTurn ? chassisSpeeds.omegaRadiansPerSecond * 3.0 : // TODO: magic number, please remove
						gyroPID.calculate(getGyro().getRadians(), targetAngle),
				getGyro());

		SwerveModuleState[] states = drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, robotMaxSpeed);

		if (manualTurn) {
			targetAngle = getGyro().getRadians() + (chassisSpeeds.omegaRadiansPerSecond / 2.0); // TODO: magic number
		}
		for (int i = 0; i < pods.length; i++) {
			pods[i].setPodState(states[i]);
		}
	}

	@Override
	public void periodic() {
		odometer.update(getGyro(), getModulePositions());
		SmartDashboard.putNumber("pigeon", getGyro().getDegrees());
		field2d.setRobotPose(odometer.getPoseMeters()
				.transformBy(new Transform2d(new Translation2d(), new Rotation2d(odoAngleOffset + Math.PI))));

		// uncomment these lines for azimuth tuning
		// leftPod.setPID(azimuthkS.getDouble(SkywarpConfig.azimuthkS),
		// azimuthP.getDouble(SkywarpConfig.azimuthkP),
		// azimuthI.getDouble(SkywarpConfig.azimuthkI),
		// azimuthD.getDouble(SkywarpConfig.azimuthkD), 1,
		// ADMult.getDouble(SkywarpConfig.azimuthMaxOutput));
		// rightPod.setPID(azimuthkS.getDouble(SkywarpConfig.azimuthkS),
		// azimuthP.getDouble(SkywarpConfig.azimuthkP),
		// azimuthI.getDouble(SkywarpConfig.azimuthkI),
		// azimuthD.getDouble(SkywarpConfig.azimuthkD), 1,
		// ADMult.getDouble(SkywarpConfig.azimuthMaxOutput));
	}

	private String getFomattedPose() {
		var pose = odometer.getPoseMeters();
		return String.format(
				"(%.3f, %.3f) %.2f degrees",
				pose.getX(), pose.getY(), pose.getRotation().plus(new Rotation2d(odoAngleOffset)).getDegrees());
	}

	public void addDashboardWidgets(ShuffleboardTab tab) {
		tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
		tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
	}
}
