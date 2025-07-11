// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.RobotMap.CameraName;
import frc.robot.util.Triplet;
import frc.robot.util.Tuple;

public class PhotonVision extends SubsystemBase {

    private VisionSystemSim visionSim;

    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;

    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    List<TimestampedObject<Pose2d>> timestampedMasterPoses = new ArrayList<>();

    private StructPublisher<Pose2d> posePublisher;
    private DoublePublisher distPublisher;
    private StructSubscriber<Pose2d> masterPoseSubscriber;

    private HashMap<Double, Pose2d> masterPoses = new HashMap<>();

    public Pose2d pose = new Pose2d();

    private TurdSwerve drivetrain;

    private CameraThread camThread;

    public PhotonVision(TurdSwerve drivetrain, CameraName camName) {
        this.drivetrain = drivetrain;

        // try {
            // Attempt to load the AprilTag field layout from the specified JSON file
            // tagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("tags.json"));
            camThread = new CameraThread(camName, RobotConstants.SLAVE_CAMERA_LOCATION);
            camThread.start();

        // } catch (Exception e) {
        //     // If the file is not found or there's an error, print a message
        //     System.out.println("Failed to load AprilTag field layout");
        //     return;
        // }


        if(!Constants.IS_MASTER) {
            //telemetry
            String tab = Constants.currentRobot.toString();

            distPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable("slave").getDoubleTopic("distance").publish();
            masterPoseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.RobotType.master.toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());
            posePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable("slave").getStructTopic("full pose", Pose2d.struct).publish();
        }

        //TODO: implement simulation
        // if (!Robot.isReal()) {
        //     visionTarget = new VisionTargetSim(VisionConstants.targetPose, VisionConstants.targetModel);
        //     cameraProp = new SimCameraProperties();
        //     visionSim = new VisionSystemSim("test");

        //     visionSim.addVisionTargets(visionTarget);
        //     visionSim.addAprilTags(VisionConstants.tagLayout);

        //     // cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        //     // cameraProp.setCalibError(0.25, 0.08);
        //     // cameraProp.setFPS(20);
        //     // cameraProp.setAvgLatencyMs(35);
        //     // cameraProp.setLatencyStdDevMs(5);

        //     leftCameraSim = new PhotonCameraSim(camThread.getCameraObject(), cameraProp);
        //     rightCameraSim = new PhotonCameraSim(rightThread.getCameraObject(), cameraProp);

        //     visionSim.addCamera(leftCameraSim, VisionConstants.robotLeftToCamera);
        //     visionSim.addCamera(rightCameraSim, VisionConstants.robotRightToCamera);

        //     // Enable the raw and processed streams. These are enabled by default.
        //     leftCameraSim.enableRawStream(true);
        //     leftCameraSim.enableProcessedStream(true);

        //     rightCameraSim.enableRawStream(true);
        //     rightCameraSim.enableProcessedStream(true);

        //     // Enable drawing a wireframe visualization of the field to the camera streams.
        //     // This is extremely resource-intensive and is disabled by default.
        //     // cameraSim.enableDrawWireframe(true);
        // }
    }

    /**
     * Switch the pipeline of a camera
     *
     * @param camera   Camera to switch the pipeline of
     * @param pipeline Pipeline to switch to
     */
    public void switchPipelines(PhotonCamera camera, int pipeline) {
        camera.setPipelineIndex(pipeline);

        /*
         * if pipeline is switched out of 3d mode, the camera's thread should be paused, then resumed when switched back
         * this is to prevent the camera from trying to update the pose when it's unable to
         * error handling should catch this, but it's better to be safe than sorry
         *
         * threads should be paused with Thread.wait() and resumed with Thread.notify()
         */
    }

    // /**
    //  * check if the camera has a target using networktables
    //  * @param camera - the camera to check
    //  * @return boolean - if the camera has a target
    //  */
    // public boolean hasTarget(CameraName camera) {
    //     if(isCameraInitialized(camera)) {
    //         switch(camera) {
    //             case front:
    //                 return camThread.getCameraObject().getCameraTable().getEntry("hasTarget").getBoolean(false);
    //             default:
    //                 return false;
    //         }
    //     } else {
    //         return false;
    //     }
    // }
    
    private boolean isCameraInitialized(CameraName camName) {
        return camThread.cameraInitialized;
    }

    private synchronized void updateVision() {
        Tuple<Transform2d, Double> updates = camThread.getUpdates();
        timestampedMasterPoses.addAll(List.of(masterPoseSubscriber.readQueue()));

        //trim timestampedMasterPoses to 10 values (for efficiency)
        if (timestampedMasterPoses.size() > 10) {
            timestampedMasterPoses = timestampedMasterPoses.subList(timestampedMasterPoses.size() - 10, timestampedMasterPoses.size());
        }

        //find the master pose with the closest timestamp to the camera's timestamp
        //NT timestamps are measured in microseconds, PV timestamps are seconds.
        double visionTimeStampMicroSeconds = updates.v * 1000000d;
        Pose2d closestMasterPose = timestampedMasterPoses.stream()
            .min((a, b) -> Double.compare(Math.abs(a.serverTime - visionTimeStampMicroSeconds), Math.abs(b.serverTime - visionTimeStampMicroSeconds)))
            .map(pose -> pose.value)
            .orElse(null);
        
        //add the distance from the center of the master robot to the tag
        // closestMasterPose = new Pose2d(closestMasterPose.getTranslation().plus(new Translation2d(0d, -0.2102)), closestMasterPose.getRotation().plus(new Rotation2d(Degrees.of(180))));
        // Pose2d fieldRelativePose = closestMasterPose.plus(new Transform2d(new Translation2d(updates.k.getX(), -updates.k.getY()).rotateBy(updates.k.getRotation()), updates.k.getRotation()));
        // fieldRelativePose = new Pose2d(fieldRelativePose.getX(), fieldRelativePose.getY(), fieldRelativePose.getRotation().plus(Rotation2d.kCW_90deg));


        // closestMasterPose = new Pose2d(closestMasterPose.getTranslation().plus(new Translation2d(0d, -0.2102)), closestMasterPose.getRotation().plus(new Rotation2d(Degrees.of(180))));
        // updates.k.plus(new Transform2d(0d, -0.2102, new Rotation2d(Degrees.of(-90))));
        // Pose2d fieldRelativePose = closestMasterPose.plus(visionTranslation);
        
        Transform2d visionTranslation = new Transform2d(new Translation2d(-updates.k.getY(), -(updates.k.getX() + 0.2102)).rotateBy(drivetrain.getPose().getRotation()), updates.k.getRotation().minus(new Rotation2d(Degrees.of(90))));
        Pose2d fieldRelativePose = new Pose2d(closestMasterPose.getTranslation().plus(visionTranslation.getTranslation()),
        visionTranslation.getRotation().plus(closestMasterPose.getRotation()));
        // fieldRelativePose = new Pose2d(fieldRelativePose.getX(), fieldRelativePose.getY(), fieldRelativePose.getRotation().plus(Rotation2d.kCW_90deg));


        // add the master pose to the translation to get field-relative pose. 
        // grab the timestamp
        // grab the distance to the best tag
        drivetrain.addVisionMeasurement(fieldRelativePose, updates.v, updates.k.getTranslation().getNorm());

        posePublisher.accept(fieldRelativePose);
        pose = fieldRelativePose;

        distPublisher.accept(masterPoseSubscriber.getAtomic().serverTime / 1000000d);

    }


    private class CameraThread extends Thread {
        // private EstimatedRobotPose pose = new EstimatedRobotPose(new Pose3d(), 0, List.of(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        // private PhotonPoseEstimator poseEstimator;
        private PhotonCamera camera;
        private final Transform3d cameraPosition;
        private Double averageDistance = 0d;
        private CameraName camName;
        private Tuple<Transform2d, Double> updates;
        private boolean hasTarget = false;

        private  BooleanPublisher hasTargetPublisher;
        private  DoublePublisher targetsFoundPublisher;
        private  DoublePublisher timestampPublisher;
        // private final DoublePublisher distancePublisher;
        private  StructPublisher<Pose3d> posePublisher;

        public boolean cameraInitialized = false;

        CameraThread(CameraName camName, Transform3d cameraPosition) {
            this.camName = camName;
            this.cameraPosition = cameraPosition;

            initializeCamera();

            if(Constants.IS_MASTER) return;

            // poseEstimator = new PhotonPoseEstimator(tagLayout,
            //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);
            // poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            // tags = poseEstimator.getFieldTags();

            // poseEstimator.setFieldTags(tags);

            // Initialize NetworkTables publishers
            hasTargetPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getBooleanTopic("hasTarget").publish();
            targetsFoundPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getDoubleTopic("targetsFound").publish();
            timestampPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getDoubleTopic("timestamp").publish();
            // distancePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getDoubleTopic("distance").publish();
            posePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getStructTopic("cam pose", Pose3d.struct).publish();
        }

        @Override
        public void run() {
            if(Constants.IS_MASTER) return;
            try {
                //wait for the camera to bootup before initialization
                sleep(3000);
            } catch (InterruptedException e) {
                DataLogManager.log(camName.toString() + " sleep inital failed");
            }

            //main loop for the camera thread
            while (true) {
                if (!cameraInitialized) {
                    initializeCamera();
                } else {
                    try {
                        Transform3d robotToTag = new Transform3d();
                        double timestamp = 0;

                        // main call to grab a set of results from the camera
                        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

                        double numberOfResults = results.size(); //double to prevent integer division errors
                        double totalDistances = 0;
                        boolean hasTarget = false;

                        //fundamentally, this loop updates the pose and distance for each result. It also logs the data to shuffleboard
                        //this is done in a thread-safe manner, as global variables are only updated at the end of the loop (no race conditions)
                        for (PhotonPipelineResult result : results) {
                            if (result.hasTargets()) {
                                // the local hasTarget variable will turn true if ANY PipelineResult within this loop has a target
                                hasTarget = true;

                                // grabs the best target from the result and sends to pose estimator, iFF the pose ambiguity is below a (hardcoded) threshold
                                if (!(result.getBestTarget().getPoseAmbiguity() > 0.5)) {
                                    // poseEstimator.update(result).ifPresentOrElse(((pose) -> this.pose = pose), () -> {
                                    //     DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " pose not updated");
                                    // });

                                    robotToTag = result.getBestTarget().getBestCameraToTarget().plus(cameraPosition);
                                    // robotToTag = new Transform3d(robotToTag.getTranslation().plus(cameraPosition.getTranslation()), robotToTag.getRotation().plus(cameraPosition.getRotation()));
                                    timestamp = result.getTimestampSeconds();
                                } else {
                                    DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " pose ambiguity is high");
                                }

                                // grabs the distance to the best target (for the latest set of result)
                                totalDistances += result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();

                                hasTargetPublisher.set(true);
                                targetsFoundPublisher.set(numberOfResults);
                                timestampPublisher.set(result.getTimestampSeconds());
                                // distancePublisher.set(totalDistances / numberOfResults);
                                posePublisher.set(Pose3d.kZero.plus(robotToTag));
                            } else {
                                hasTargetPublisher.set(false);
                                targetsFoundPublisher.set(0);
                            }
                        }

                        // averages distance over all results
                        averageDistance = totalDistances / numberOfResults;
                        updates = new Tuple<Transform2d, Double>(new Transform2d(robotToTag.getTranslation().toTranslation2d(), robotToTag.getRotation().toRotation2d()), timestamp);
                        this.hasTarget = hasTarget;
                        if (hasTarget) {
                            updateVision();
                        }
                    } catch (IndexOutOfBoundsException e) {
                        // if there are no results,
                        // LightningShuffleboard.setBool("Vision", camName.toString() + " functional", false);
                        // LightningShuffleboard.setBool("Vision", camName.toString() + " hasTarget", false);
                        this.hasTarget = false;
                    }
                    try {
                        sleep(5);
                    } catch (InterruptedException e) {
                        DataLogManager.log(camName.toString() + " sleep failed");
                    }
                }
            }
        }

        /**
         * Returns the most recent pose and average distance to the best target <p>
         *
         * using a tuple as a type-safe alternative to the classic "return an array" (i hate java) <p>
         * this is also thread-safe, and will onlu return the most recent values from the same timestamp <p>
         *
         * @return Tuple<EstimatedRobotPose, Double> - the most recent pose and average distance to the best target
         */
        public Tuple<Transform2d, Double> getUpdates() {
            return updates;
        }

        /**
         * Returns if the camera (during latest loop cycle) has a target
         *
         * This is separate from the value on the NetworkTables, as this value is updated for the entire loop cycle <p>
         * there is an edge case where a target is found, but the pose is not updated <p>
         * likewise, there is an edge case where a target is found and then lost, but the pose is updated <p>
         * to solve this, the hasTarget value is marked as true iff any update has been sent to the estimator <p>
         * @ImplNote this is NOT strictly synchronized with the value from {@link #getUpdates()}; be careful when using this value. should use PhotonVision's hasTarget() function for most cases
         * @return has a target or not
         */
        public boolean hasTarget() {
            return hasTarget;
        }

        public PhotonCamera getCameraObject() {
            return camera;
        }

        private void initializeCamera() {
            try {
                camera = new PhotonCamera(camName.toString());
                cameraInitialized = true;
            } catch (Exception e) {
                DataLogManager.log("warning: camera not initialized");
            }
        }
    }



    /**
     * multicam imp'l of {@link #updateVision()}
     * @param caller - the camera that called the function, used to determine which camera's pose to use
     * @deprecated
     */
    @Deprecated
    private synchronized void updateVision(CameraName caller) {
        // Tuple<EstimatedRobotPose, Double> leftUpdates = camThread.getUpdates();
        // Tuple<EstimatedRobotPose, Double> rightUpdates = rightThread.getUpdates();

        // final double maxAcceptableDist = 4d;
        // boolean shouldUpdateLeft = true;
        // boolean shouldUpdateRight = true;


        // // prefer the camera that called the function (has known good values)
        // // if the other camera has a target, prefer the one with the lower distance to best tag
        // switch (caller) {
        //     case LEFT:
        //         if((rightThread.hasTarget() && rightUpdates.v < leftUpdates.v) && shouldUpdateRight) {
        //             drivetrain.addVisionMeasurement(rightUpdates.k, rightUpdates.v);
        //         } else if (shouldUpdateLeft) {
        //             drivetrain.addVisionMeasurement(leftUpdates.k, leftUpdates.v);
        //         }
        //     break;
        //     case RIGHT:
        //         if((camThread.hasTarget() && leftUpdates.v < rightUpdates.v) && shouldUpdateLeft) {
        //             drivetrain.addVisionMeasurement(leftUpdates.k, rightUpdates.v);
        //         } else if (shouldUpdateRight) {
        //             drivetrain.addVisionMeasurement(rightUpdates.k, rightUpdates.v);
        //         }
        //     break;
        // }
    }

}
