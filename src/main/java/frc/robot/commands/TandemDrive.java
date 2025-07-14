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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TurdConstants;
import frc.robot.TurdConstants.RobotConfig;
import frc.robot.subsystems.TurdSwerve;

public class TandemDrive extends Command {
    private final TurdSwerve swerve;
    private final PIDController anglePID = new PIDController(0.2, 0, 0);
    private final PIDController xPID = new PIDController(0.3, 0, 0);
    private final PIDController yPID = new PIDController(0.3, 0, 0);
	private Supplier<Pose2d> tandemTarget;

    private Pose2d targetPose = new Pose2d();

    private DoubleEntry kP;
    private DoubleEntry kI;
    private DoubleEntry kD;
    private DoubleEntry kP_angle;
    private DoubleEntry kI_angle;
    private DoubleEntry kD_angle;
    private StructEntry<Pose2d> targetPosePublisher;

    public TandemDrive(TurdSwerve swerve, Supplier<Pose2d> tandemTarget) {
        this.swerve = swerve;
        this.tandemTarget = tandemTarget;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        anglePID.enableContinuousInput(0, Math.PI * 2);

        kP = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kP").getEntry(0.0);
        kI = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kI").getEntry(0.0);
        kD = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kD").getEntry(0.0);

        kP_angle = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kP_angle").getEntry(0.0);
        kI_angle = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kI_angle").getEntry(0.0);
        kD_angle = NetworkTableInstance.getDefault().getTable("TandemDrive").getDoubleTopic("kD_angle").getEntry(0.0);


        targetPosePublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getStructTopic("targetPose", Pose2d.struct).getEntry(new Pose2d());

        if(Constants.IS_MASTER) {
            //reset the initial pose to the robot-system pose
            Pose2d robotTargetPose = targetPose.plus(new Transform2d(TurdConstants.RobotConfig.offsetPosition, Rotation2d.kZero));
            swerve.resetPose(robotTargetPose);

            // kP.accept(0);
            // kI.accept(0);
            // kD.accept(0);

            // kP_angle.accept(0);
            // kI_angle.accept(0);
            // kD_angle.accept(0);
        }
    }

    @Override
    public void execute() {
        Pose2d robotTargetPose = tandemTarget.get().plus(new Transform2d(TurdConstants.RobotConfig.offsetPosition, Rotation2d.kZero));


        Pose2d currentPose = swerve.getPose();
        double xOut = xPID.calculate(currentPose.getX(), robotTargetPose.getX());
        double yOut = yPID.calculate(currentPose.getY(), robotTargetPose.getY());
        double rOut = anglePID.calculate(currentPose.getRotation().getRadians(), robotTargetPose.getRotation().getRadians());

        swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rOut, currentPose.getRotation()));

        // anglePID.setPID(kP_angle.get(), kI_angle.get(), kD_angle.get());
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
