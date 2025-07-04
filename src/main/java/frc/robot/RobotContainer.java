// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PointAndDrive;
import frc.robot.commands.SpinManually;
import frc.robot.subsystems.DiffySwerve;
import frc.robot.subsystems.TurdSwerve;
import frc.robot.subsystems.InputGetter;
import frc.robot.subsystems.InputSender;
import frc.robot.subsystems.PhotonVision;

public class RobotContainer {
	public static final CommandXboxController joystick = new CommandXboxController(JoystickConstants.driverPort);
	public static final TurdSwerve swerve = new TurdSwerve(0);
    private InputGetter inputGetter;
    private PhotonVision photonVision;

	public RobotContainer() {
        inputGetter = new InputGetter();
        if(Constants.IS_MASTER) {
            new InputSender();
            new PhotonVision();
        }


		//manually spin individual pods based on dpad input + right joystick input
		joystick.pov(-1).onFalse(new SpinManually(swerve, () -> getPodToTest(), () -> joystick.getRightX()));

		//point all pods at an angle based on right joystick input
		joystick.leftTrigger().whileTrue(new PointAndDrive(swerve, () -> new Translation2d(joystick.getRightX(), joystick.getRightY()), () -> joystick.getRightTriggerAxis()));

		joystick.start().whileTrue(new InstantCommand(swerve::resetPods, swerve));


		//drive bindings
		Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(joystick.getRightX(),
				joystick.getRightY());
		Supplier<Translation2d> driverLeftJoystick = () -> new Translation2d(joystick.getLeftX(),
				joystick.getLeftY());
		swerve.setDefaultCommand(
				new DriveCommand(swerve, driverLeftJoystick, driverRightJoystick));


		// PID tuning/testing function. just sets FL pod to DPAD angle.
		// swerve.setDefaultCommand(new RunCommand(() -> swerve.getPods().get(0).setPodState(new SwerveModuleState(joystick.getLeftY(), Rotation2d.fromDegrees(joystick.getHID().getPOV()))), swerve));
	}
	
	private int getPodToTest() {
		switch (joystick.getHID().getPOV()) {
			case 0:
				return 0; //FL
			case 90:
				return 1; //FR
			case 180:
				return 3; //BR
			case 270:
				return 2; //BL
			default:
				return -1;
		}
		
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
