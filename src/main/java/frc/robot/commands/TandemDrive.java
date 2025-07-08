// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TurdConstants;
import frc.robot.TurdConstants.RobotConfig;
import frc.robot.subsystems.TurdSwerve;

public class TandemDrive extends Command {
    private final TurdSwerve swerve;
    private final PIDController anglePID = new PIDController(0.1, 0, 0);
    private final PIDController xPID = new PIDController(0.12, 0, 0);
    private final PIDController yPID = new PIDController(0.12, 0, 0);
	private Supplier<Translation2d> joystickRight, joystickLeft;

    private Pose2d targetPose = new Pose2d();

    private DoubleEntry kP;
    private DoubleEntry kI;
    private DoubleEntry kD;
    private StructEntry<Pose2d> targetPosePublisher;

    public TandemDrive(TurdSwerve swerve, Supplier<Translation2d> joystickLeft, Supplier<Translation2d> joystickRight) {
        this.swerve = swerve;
        this.joystickLeft = joystickLeft;
        this.joystickRight = joystickRight;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        anglePID.enableContinuousInput(0, Math.PI * 2);

        // kP = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kP").getEntry(0.0);
        // kI = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kI").getEntry(0.0);
        // kD = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kD").getEntry(0.0);

        // kP.accept(0);
        // kI.accept(0);
        // kD.accept(0);

        targetPosePublisher = NetworkTableInstance.getDefault().getTable("TandemDrive").getStructTopic("targetPose", Pose2d.struct).getEntry(new Pose2d());

        //reset the initial pose to the robot-system pose
        Pose2d robotTargetPose = targetPose.plus(new Transform2d(TurdConstants.RobotConfig.offsetPosition, Rotation2d.kZero));
        swerve.resetPose(targetPose);
    }

    @Override
    public void execute() {
        Transform2d transform = new Transform2d(
            new Translation2d(-joystickRight.get().getY(), -joystickRight.get().getX()), new Rotation2d(-joystickLeft.get().getX())
        ).times(Robot.kDefaultPeriod).times(RobotConfig.robotMaxSpeed);

        targetPose = targetPose.plus(transform);
        Pose2d robotTargetPose = targetPose.plus(new Transform2d(TurdConstants.RobotConfig.offsetPosition, Rotation2d.kZero));


        Pose2d currentPose = swerve.getPose();
        double xOut = xPID.calculate(currentPose.getX(), robotTargetPose.getX());
        double yOut = yPID.calculate(currentPose.getY(), robotTargetPose.getY());
        double rOut = anglePID.calculate(currentPose.getRotation().getRadians(), robotTargetPose.getRotation().getRadians());

        swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rOut, currentPose.getRotation()));

        // anglePID.setPID(kP.get(), kI.get(), kD.get());
        // xPID.setPID(kP.get(), kI.get(), kD.get());
        // yPID.setPID(kP.get(), kI.get(), kD.get());

        targetPosePublisher.accept(robotTargetPose);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
