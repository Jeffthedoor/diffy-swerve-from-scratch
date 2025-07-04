// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.InputInterface;

public class InputSender extends SubsystemBase {
	/** Creates a new SendInputs. */
	private XboxController controller;

	public InputSender() {
		controller = new XboxController(0);
	}

	@Override
	public void periodic() {
		InputInterface.updateInputs(controller, DriverStation.isEnabled(), Timer.getFPGATimestamp());
	}
}
