// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.TurdSwerve;

public class FindInitialOffset extends Command {
    private Timer timer = new Timer();
    private int i = 0;
    private Matrix<N4, N4> poseMatrix = new Matrix<N4, N4>(N4.instance, N4.instance); 

    private final PhotonVision vision;
    private final TurdSwerve swerve;

    private final StructPublisher<Pose3d> finalPublisher;
    private final StructPublisher<Pose3d> guessPublisher;
    
    public FindInitialOffset(PhotonVision vision, TurdSwerve swerve) {
        this.vision = vision;
        this.swerve = swerve;

        finalPublisher = NetworkTableInstance.getDefault().getTable("Initial Guess").getStructTopic("final pose", Pose3d.struct).publish();
        guessPublisher = NetworkTableInstance.getDefault().getTable("Initial Guess").getStructTopic("guessed poses", Pose3d.struct).publish();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Pose3d visionPose = vision.pose;

        guessPublisher.accept(visionPose);
        poseMatrix.plus(visionPose.toMatrix());
        i++;

    }

    @Override
    public void end(boolean interrupted) {
        Pose3d finalPose = new Pose3d(poseMatrix.div(i));
        swerve.setInitialGuess(finalPose.toPose2d());
        finalPublisher.accept(finalPose);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }
}
