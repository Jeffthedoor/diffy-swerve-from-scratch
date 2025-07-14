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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RobotMap.CameraName;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PointAndDrive;
import frc.robot.commands.SpinManually;
import frc.robot.commands.TandemDrive;
import frc.robot.subsystems.DiffySwerve;
import frc.robot.subsystems.TurdSwerve;
import frc.robot.subsystems.InputGetter;
import frc.robot.subsystems.InputSender;
import frc.robot.subsystems.PhotonVision;

public class RobotContainer {
	public final TurdSwerve swerve;
    private InputGetter inputGetter;
    private PhotonVision photonVision;

	public RobotContainer() {
        inputGetter = new InputGetter();
        if(Constants.IS_MASTER) {
            new InputSender();
			swerve = new TurdSwerve(0);
			photonVision = new PhotonVision(swerve, CameraName.master);
        } else {
			swerve = new TurdSwerve(1);
			photonVision = new PhotonVision(swerve, CameraName.slave);
		}



		//manually spin individual pods based on dpad input + right joystick input
		new Trigger(() -> (inputGetter.getPOV() == -1)).onFalse(new SpinManually(swerve, () -> getPodToTest(), () -> inputGetter.getRightX()));

		//point all pods at an angle based on right joystick input
		new Trigger(inputGetter::getLeftBumper).whileTrue(new PointAndDrive(swerve, () -> new Translation2d(inputGetter.getRightX(), inputGetter.getRightY()), () -> inputGetter.getRightTriggerAxis()));

		new Trigger(inputGetter::getStartButton).whileTrue(new InstantCommand(swerve::resetPods, swerve));


		//drive bindings
		Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(inputGetter.getRightX(),
				inputGetter.getRightY());
		Supplier<Translation2d> driverLeftJoystick = () -> new Translation2d(inputGetter.getLeftX(),
				inputGetter.getLeftY());
		// swerve.setDefaultCommand(
		// 		new DriveCommand(swerve, driverLeftJoystick, driverRightJoystick));

		swerve.setDefaultCommand(new TandemDrive(swerve, inputGetter::getTandemTarget).ignoringDisable(true));


		// PID tuning/testing function. just sets FL pod to DPAD angle.
		// swerve.setDefaultCommand(new RunCommand(() -> swerve.getPods().get(0).setPodState(new SwerveModuleState(joystick.getLeftY(), Rotation2d.fromDegrees(joystick.getHID().getPOV()))), swerve));
	}
	
	private int getPodToTest() {
		switch (inputGetter.getPOV()) {
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
