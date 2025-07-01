// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMap.PodConfig;
import frc.robot.util.TunableNumber;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.RobotConfig;

public class DiffySwerve extends SubsystemBase {
	private final Pigeon2 gyro;

	private Pigeon2SimState gyroSimState;

	private final ArrayList<DrivePod> pods = new ArrayList<DrivePod>();

	private final SwerveDriveOdometry odometer;
	private final SwerveDrivePoseEstimator poseEstimator;

	private TunableNumber azimuthkP = new TunableNumber("azimuthkP", PodConfig.kP, "PID");
	private TunableNumber azimuthkI = new TunableNumber("azimuthkI", PodConfig.kI, "PID");
	private TunableNumber azimuthkD = new TunableNumber("azimuthkD", PodConfig.kD, "PID");

	public double targetAngle = 0;
	private double odoAngleOffset = Math.PI * 0.0;

	private Rotation2d gyroResetAngle = new Rotation2d();

	private SwerveDriveKinematics drivetrainKinematics;
	private final double robotMaxSpeed;

	private final Field2d field2d = new Field2d();

	StructArrayPublisher<SwerveModuleState> SMSPublisher;
	StructPublisher<Pose2d> PosePublisher;
	StructPublisher<ChassisSpeeds> ChassisSpeedsPublisher;

	public DiffySwerve() {
		this.drivetrainKinematics = RobotMap.drivetrainKinematics;
		this.robotMaxSpeed = RobotConfig.robotMaxSpeed;

		gyro = new Pigeon2(RobotMap.pigeonID);


		for (int i = 0; i < RobotMap.PodConfigs.length; i++) {
			pods.add(new DrivePod(i, RobotMap.PodConfigs[i].encoderID, RobotMap.PodConfigs[i].leftMotorID, RobotMap.PodConfigs[i].rightMotorID, PodConfig.leftMotorInvert, PodConfig.rightMotorInvert, RobotMap.PodConfigs[i].encoderOffset, PodConfig.encoderInvert, PodConfig.ampLimit, PodConfig.motorsBrake, PodConfig.rampRate, azimuthkP.doubleValue(), azimuthkI.doubleValue(), azimuthkD.doubleValue(), getGlobalOutputScalar(), PodConfig.motorGearing));
		}
		
		SwerveModulePosition positions[] = pods.stream()
				.map(DrivePod::getPodPosition)
				.toArray(SwerveModulePosition[]::new);

		odometer = new SwerveDriveOdometry(drivetrainKinematics, new Rotation2d(0), positions);
		poseEstimator = new SwerveDrivePoseEstimator(
					drivetrainKinematics,
					getGyro(),
					getModulePositions(),
					new Pose2d());

		// gyro.configAllSettings(new Pigeon2Configuration());

		SMSPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
		PosePublisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
		ChassisSpeedsPublisher = NetworkTableInstance.getDefault().getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();

		if(Robot.isSimulation()) {
			gyroSimState = new Pigeon2SimState(gyro);
		}
	}

	public void resetOdometry(Pose2d pose) {
		odoAngleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI * 0.5 : Math.PI * 1.5;
		odometer.resetPosition(new Rotation2d(odoAngleOffset), getModulePositions(), pose);
	}

	public SwerveModulePosition[] getModulePositions() {
		return pods.stream()
				.map(DrivePod::getPodPosition)
				.toArray(SwerveModulePosition[]::new);
	}

	public SwerveModuleState[] getModuleStates() {
		return pods.stream()
				.map(DrivePod::getState)
				.toArray(SwerveModuleState[]::new);
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
		return gyro.getRotation2d().minus(gyroResetAngle);
	}

	public void resetGyro() {
		gyroResetAngle = getGyro().plus(gyroResetAngle);
		targetAngle = 0;
	}

	public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
		chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
				chassisSpeeds.vyMetersPerSecond,
				chassisSpeeds.omegaRadiansPerSecond * 3.0, // TODO: magic number, please remove
				getGyro());

		SwerveModuleState[] states = drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, robotMaxSpeed);
		if (isTurning()) {
			if (isMoving()) {
				for (int i = 0; i < pods.size(); i++) {
					pods.get(i).setPodState(new SwerveModuleState(0, pods.get(i).getState().angle), false); // TODO: has bug
					// pods[i].setPodState(new SwerveModuleState(0, pods[i].getState().angle)); // stop driving and don't change the azimuth angle; doesn't work because it changes the pod's "target angle" which disables "isTurning()" check
				}
			} else {
				for (int i = 0; i < pods.size(); i++) {
					pods.get(i).setPodState(new SwerveModuleState(0, states[i].angle)); // stop driving, but correct the azimuth angle
				}
			}
		} else {
			for (int i = 0; i < pods.size(); i++) {
				pods.get(i).setPodState(states[i]);
			}
		}
	}

	public DrivePod[] getPods() {
		return (DrivePod[]) pods.toArray();
	}

	@Override
	public void simulationPeriodic() {
		//update the gyro to functionally be the robot/odo angle
		gyroSimState.setAngularVelocityZ(RobotMap.drivetrainKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);
	}

	@Override
	public void periodic() {	

		odometer.update(getGyro(), getModulePositions());
		SmartDashboard.putNumber("pigeon", getGyro().getDegrees());
		field2d.setRobotPose(odometer.getPoseMeters()
				.transformBy(new Transform2d(new Translation2d(), new Rotation2d(odoAngleOffset + Math.PI))));

		SMSPublisher.set(getModuleStates());
		PosePublisher.set(odometer.getPoseMeters());
		ChassisSpeedsPublisher.set(drivetrainKinematics.toChassisSpeeds(getModuleStates()));


		if(Constants.tuningMode) {
			// update the tunable numbers
			if (azimuthkP.hasChanged() || azimuthkI.hasChanged() || azimuthkD.hasChanged()) {
				setPIDs();
			}
		}


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

	private void setPIDs() {
		pods.forEach(pod -> {
			pod.setPID(
				azimuthkP.doubleValue(),
				azimuthkI.doubleValue(),
				azimuthkD.doubleValue()
			);
		});
	}

	/**
	 * Returns the maximum output scalar of all pods.
	 * @return A DoubleSupplier that provides the maximum output scalar of all pods.
	 */
	private DoubleSupplier getGlobalOutputScalar() {
		return () -> pods.stream()
				.mapToDouble(DrivePod::getPodOutputScalar)
				.max()
				.orElse(0);
	}

	public void addVisionMeasurement(EstimatedRobotPose visionPose, double distance) {
		poseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(visionPose.timestampSeconds), VecBuilder.fill(distance / 2, distance / 2, distance / 2));
	}

	private boolean isTurning() {
		for (DrivePod pod : pods) {
			if (pod.isTurning()) {
				return true;
			}
		}
		return false;
	}
	
	private boolean isMoving() {
		for (DrivePod pod : pods) {
			if (pod.isMoving()) {
				return true;
			}
		}
		return false;
	}
}
