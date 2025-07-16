// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Robot;
import frc.robot.TurdConstants;
import frc.robot.TurdConstants.RobotConfig;
import frc.robot.util.InputInterface;

public class InputSender extends SubsystemBase {
	/** Creates a new SendInputs. */
	private XboxController controller;
	private SlewRateLimiter xLimiter = new SlewRateLimiter(1.0);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(1.0);
	private SlewRateLimiter rotationalLimiter = new SlewRateLimiter(3.0);


	public InputSender() {
		controller = new XboxController(JoystickConstants.driverPort);		
	}

	@Override
	public void periodic() {
		InputInterface.updateInputs(controller, DriverStation.isEnabled(), Timer.getFPGATimestamp(), grabJoystickVelocity());
	}

	private Pose2d grabJoystickVelocity() {
        return new Pose2d(new Translation2d(xLimiter.calculate(controller.getRightY()), yLimiter.calculate(controller.getRightX())).times(-0.2), new Rotation2d(rotationalLimiter.calculate(controller.getLeftX())).times(-0.1));
	}
}
