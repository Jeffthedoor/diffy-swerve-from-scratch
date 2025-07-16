// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.RobotConstants;
import frc.robot.TurdConstants.RobotConfig;
import frc.robot.subsystems.TurdSwerve;
import frc.robot.util.TunableNumber;

public class TandemDrive extends Command {
    private final TurdSwerve swerve;
	private Supplier<Pose2d> tandemTarget;

    private Pose2d targetPose = new Pose2d();

    private StructEntry<Pose2d> targetPosePublisher;


    //PID values
    private TunableNumber kP = new TunableNumber("tandem kP", RobotConstants.tandemkP);
    private TunableNumber kI = new TunableNumber("tandem kI", RobotConstants.tandemkI);
    private TunableNumber kD = new TunableNumber("tandem kD", RobotConstants.tandemkD);
    private TunableNumber kP_angle = new TunableNumber("tandem kP_angle", RobotConstants.tandemkP_angle);
    private TunableNumber kI_angle = new TunableNumber("tandem kI_angle", RobotConstants.tandemkI_angle);
    private TunableNumber kD_angle = new TunableNumber("tandem kD_angle", RobotConstants.tandemkD_angle);


    private final PIDController anglePID = new PIDController(kP_angle.getDefault(), kI_angle.getDefault(), kD_angle.getDefault());
    private final PIDController xPID = new PIDController(kP.getDefault(), kI.getDefault(), kD.getDefault());
    private final PIDController yPID = new PIDController(kP.getDefault(), kI.getDefault(), kD.getDefault());

    public TandemDrive(TurdSwerve swerve, Supplier<Pose2d> tandemTarget) {
        this.swerve = swerve;
        this.tandemTarget = tandemTarget;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        anglePID.enableContinuousInput(0, Math.PI * 2);

        
        if(Constants.IS_MASTER) {
            //reset the initial pose to the robot-system pose
            // only do this if on the master robot, as slaves are synced to master automatically
            Pose2d robotTargetPose = targetPose.plus(new Transform2d(RobotConfig.offsetPosition, Rotation2d.kZero));
            swerve.resetPose(robotTargetPose);
        }

        //initialize telemetry
        targetPosePublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getStructTopic("targetPose", Pose2d.struct).getEntry(new Pose2d());
    }

    @Override
    public void execute() {
        //grab the target pose calculated by the master robot, add it onto the offset position for this robot
        Pose2d robotTargetPose = tandemTarget.get().plus(new Transform2d(RobotConfig.offsetPosition, Rotation2d.kZero));

        //calculate the speeds to drive towards the target pose
        Pose2d currentPose = swerve.getPose();
        // double xOut = MathUtil.applyDeadband(xPID.calculate(currentPose.getX(), robotTargetPose.getX()),  RobotConstants.tandemTranslation_deadband);
        // double yOut = MathUtil.applyDeadband(yPID.calculate(currentPose.getY(), robotTargetPose.getY()),  RobotConstants.tandemTranslation_deadband);;
        // double rOut = MathUtil.applyDeadband(anglePID.calculate(currentPose.getRotation().getRadians(), robotTargetPose.getRotation().getRadians()),  RobotConstants.tandemTranslation_deadband);;

        double xOut = xPID.calculate(currentPose.getX(), robotTargetPose.getX());
        double yOut = yPID.calculate(currentPose.getY(), robotTargetPose.getY());
        double rOut = anglePID.calculate(currentPose.getRotation().getRadians(), robotTargetPose.getRotation().getRadians());

        swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rOut, currentPose.getRotation()));

        
        // update the PID values from the tunable numbers
        if(Constants.tuningMode) {
            anglePID.setPID(kP_angle.doubleValue(), kI_angle.doubleValue(), kD_angle.doubleValue());
            xPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
            yPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
        }

        //update telemetry
        targetPosePublisher.accept(robotTargetPose);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
