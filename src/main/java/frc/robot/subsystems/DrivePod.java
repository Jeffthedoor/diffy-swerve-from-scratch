// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.RobotMap;

/** This is a sample pod that uses a CANcoder and TalonFXes. */
public class DrivePod extends SubsystemBase {
	// public Encoder absoluteEncoder;
	private TalonFXS leftMotor;
	private TalonFXS rightMotor;
	private AnalogEncoder encoder;

	private TalonFXSSimState leftMotorSim;
	private TalonFXSSimState rightMotorSim;
	private AnalogEncoderSim absoluteEncoderSim;
	private DifferentialDrivetrainSim drivetrainSim;
	
	private double offset;

	TalonFXSConfiguration leftConfig = new TalonFXSConfiguration();
	TalonFXSConfiguration rightConfig = new TalonFXSConfiguration();

	// variable that determines whether or not to apply PID configurations to the
	// motor (defaults to true for initial application)
	// boolean apply = true;

	private final PIDController anglePID;

	public DrivePod(int absoluteEncoderID, int leftID, int rightID, boolean leftInvert, boolean rightInvert, double absoluteEncoderOffset,
			boolean encoderInvert, int ampLimit, boolean brake,
			double rampRate, double kP, double kI, double kD, double maxOut, double motorGearing) {

		leftMotor = makeMotor(leftID, leftInvert, brake, ampLimit, motorGearing, rampRate, leftConfig);
		rightMotor = makeMotor(rightID, rightInvert, brake, ampLimit, motorGearing, rampRate, rightConfig);


		encoder = new AnalogEncoder(absoluteEncoderID);
		encoder.setInverted(false);	
		this.offset = absoluteEncoderOffset;

		anglePID = new PIDController(kP, kI, kD);
		// anglePID.enableContinuousInput(0, 1);

		resetPod();

		if(Robot.isSimulation()) {
			makeSim();
		}
	}

	/**
	 * Creates a new TurdonFX (please use this for drive motors only)
	 * 
	 * @param id                         CAN ID for the motor
	 * @param inverted                   true for CW+, false for CCW+
	 * @param isBrake                    true for brake, false for coast
	 * @param statorLimit                the stator current limit in amps
	 * @param rampRate                   time it takes for the motor to reach full
	 *                                   power from zero power in seconds
	 * @param ENCODER_TO_MECHANISM_RATIO ratio between the feedback encoder
	 *                                   (integrated for drive motors) and the
	 *                                   mechanism. this varies based on your gear
	 *                                   ratio
	 * @param ROTOR_TO_ENCODER_RATIO     ratio between the rotor and the feedback
	 *                                   encoder. this is usually 1 for drive motors
	 */
	private TalonFXS makeMotor(int id, boolean inverted, boolean isBrake, double statorLimit, double motorGearing, double rampRate, TalonFXSConfiguration driveConfig) {
		// I figured nobody had the guts to put a CANivore on a turdswerve, so i'm
		// leaving out the CAN bus parameter
		TalonFXS motor = new TalonFXS(id);

		// set neutral mode and inverts
		driveConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		driveConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

		// set current limits; supply current limits are hardcoded because they are
		// almost always the same
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.SupplyCurrentLimit = 70d;
		driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 40d;
		driveConfig.CurrentLimits.SupplyCurrentLowerTime = 1d;

		driveConfig.CurrentLimits.StatorCurrentLimitEnable = statorLimit > 0;
		driveConfig.CurrentLimits.StatorCurrentLimit = statorLimit;

		// this is kind of bad code, but it's the easiest way to set a ramp rate
		// regardless of control type
		driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
		driveConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = rampRate;
		driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate;
		driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
		driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
		driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

		// set feedback ratios
		driveConfig.ExternalFeedback.SensorToMechanismRatio = motorGearing;
		driveConfig.ExternalFeedback.RotorToSensorRatio = 1d;

		motor.getConfigurator().apply(driveConfig);

		return motor;
	}

	private void makeSim() {
		leftMotorSim = leftMotor.getSimState();
		rightMotorSim = rightMotor.getSimState();
		absoluteEncoderSim = new AnalogEncoderSim(encoder);


		// left drivetrain motors are typically CCW+
		leftMotorSim.MotorOrientation = ChassisReference.CounterClockwise_Positive;

		// right drivetrain motors are typically CW+
		rightMotorSim.MotorOrientation = ChassisReference.Clockwise_Positive;

		drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
			KitbotMotor.kSingleMiniCIMPerSide, // 2 CIMs per side.
			KitbotGearing.k10p71,        // 10.71:1
			KitbotWheelSize.kSixInch,    // 6" diameter wheels.
			3,
			null                         // No measurement noise.
		);

		//TODO: ensure stall torque, free current, and stall current are correct; these values are guesstimates
		// drivetrainSim = new DifferentialDrivetrainSim(new DCMotor(16, 300, 20, 2, RotationsPerSecond.of(27).in(RadiansPerSecond), 1),
		// 		SimConstants.gearRatio,
		// 		SimConstants.inertia,
		// 		SimConstants.mass, 
		// 		SimConstants.wheelRadius,
		// 		SimConstants.trackWidth,
		// 		VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
	}


	public void resetPod() {
		rightMotor.setPosition(0);
		leftMotor.setPosition(0);
		offset = -encoder.get();
	}

	public void resetZero() {
		offset = -encoder.get();
		resetPod();
	}

	// current position of absolute encoder in rotations
	private double getAngle() {
		return encoder.get() + offset;
	}

	// average position accross drive motors
	private double getPosition() {
		return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2;
	}

	public void stop() {
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

	public SwerveModulePosition getPodPosition() {
		return new SwerveModulePosition(getPosition(),
				Rotation2d.fromRotations(getAngle()));
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(leftMotor.getVelocity().getValueAsDouble(),
				Rotation2d.fromRotations(getAngle()));
	}

	public void setPodState(SwerveModuleState state) {
		// make sure pod is inside of possible rotation zone
		while (state.angle.getRotations() > 1) {
			state = new SwerveModuleState(state.speedMetersPerSecond, state.angle.minus(Rotation2d.k180deg.plus(Rotation2d.k180deg)));
		} 
		while (state.angle.getRotations() < -1) {
			state = new SwerveModuleState(state.speedMetersPerSecond, state.angle.plus(Rotation2d.k180deg.plus(Rotation2d.k180deg)));
		} 
		while (state.angle.getRotations() > RobotMap.podRotationUpperBound) {
			state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.minus(Rotation2d.k180deg));
		} 
		while (state.angle.getRotations() < RobotMap.podRotationLowerBound) {
			state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.plus(Rotation2d.k180deg));
		} 

		// TODO: optimize pod target heading based on current heading

		//initialize outputs to raw speed
		double leftOutput = state.speedMetersPerSecond; 
		double rightOutput = state.speedMetersPerSecond; 

		//adjust outputs based on the PID Controller
		double correction = anglePID.calculate(getAngle(), state.angle.getRotations());
		leftOutput += correction;
		rightOutput -= correction;

		//normalize outputs to be between -1 and 1
		double maxOutput = Math.max(Math.abs(leftOutput), Math.abs(rightOutput));
		if (maxOutput > 1) {
			leftOutput /= maxOutput;
			rightOutput /= maxOutput;
		}
		// set the motor outputs
		leftMotor.setControl(new DutyCycleOut(leftOutput));
		rightMotor.setControl(new DutyCycleOut(rightOutput));



		//i have no idea what this stuff down here is for honestly

		// speed = Math.abs(state.speedMetersPerSecond) < .01 ? 0 : state.speedMetersPerSecond;

		// double error = (state.angle.getRadians() - absoluteEncoder.getAbsolutePosition().getValueAsDouble())
		// 		% (2 * Math.PI);
		// error = error > Math.PI ? error - 2 * Math.PI : error;
		// error = error < -Math.PI ? error + 2 * Math.PI : error;
		// error *= 180 / Math.PI;
	}

	public void setRotationalSpeed(double speed) {
		leftMotor.set(speed);
		rightMotor.set(-speed);
	}


	@Override
	public void periodic() {
		// TODO: dont use smartdashboard
		// SmartDashboard.putNumber("absolute encoder" + absoluteEncoder.getDeviceID(),
		// 		absoluteEncoder.getAbsolutePosition().getValueAsDouble());
		// SmartDashboard.putNumber("azimuth pose " + absoluteEncoder.getDeviceID(),
		// 		azimuthMotor.getPosition().getValueAsDouble());
		// SmartDashboard.putNumber("azimuth pose " + config.absoluteEncoderID,
		// azimuthMotor.);

		// SmartDashboard.putNumber("drive pos " + driveMotor.getDeviceId(),
		// driveEncoder.getPosition());
		// SmartDashboard.putNumber("azimuth.getAppliedOutput()" +
		// azimuthMotor.getDeviceId(), azimuthMotor.getAppliedOutput());
		// //getAppliedOutput());



	}

	@Override
	public void simulationPeriodic() {
		leftMotorSim.setSupplyVoltage(16);
		rightMotorSim.setSupplyVoltage(16);

		//TODO: might not have to invert the left motor voltage
		drivetrainSim.setInputs(
				leftMotorSim.getMotorVoltage(),
				rightMotorSim.getMotorVoltage());
		drivetrainSim.update(0.02); // 20ms update rate, typical for simulation
		double metersToMotorRotations = SimConstants.gearRatio / (2 * Math.PI * SimConstants.wheelRadius);
		leftMotorSim.setRawRotorPosition(drivetrainSim.getLeftPositionMeters() * metersToMotorRotations);
		rightMotorSim.setRawRotorPosition(drivetrainSim.getRightPositionMeters() * metersToMotorRotations);
		leftMotorSim.setRotorVelocity(drivetrainSim.getLeftVelocityMetersPerSecond() * metersToMotorRotations);
		rightMotorSim.setRotorVelocity(drivetrainSim.getRightVelocityMetersPerSecond() * metersToMotorRotations);

		absoluteEncoderSim.set(MathUtil.inputModulus(drivetrainSim.getHeading().getRotations(), -1,  1));
	}
}
