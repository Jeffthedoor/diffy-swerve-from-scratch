// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CameraName;
import frc.robot.util.Tuple;

public class PhotonVision extends SubsystemBase {

    private VisionSystemSim visionSim;

    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;

    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    private StructPublisher<Pose3d> posePublisher;
    private DoublePublisher distPublisher;

    public Pose3d pose = new Pose3d();

    // private Swerve drivetrain;

    private CameraThread camThread;

    public PhotonVision(CameraName camName) {
        // this.drivetrain = drivetrain;

        AprilTagFieldLayout tagLayout = null;

        try {
            // Attempt to load the AprilTag field layout from the specified JSON file
            tagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("tags.json"));
            camThread = new CameraThread(camName, new Transform3d(), tagLayout);
            camThread.start();

        } catch (Exception e) {
            // If the file is not found or there's an error, print a message
            System.out.println("Failed to load AprilTag field layout");
            return;
        }



        posePublisher = NetworkTableInstance.getDefault().getTable("Vision").getStructTopic("pose", Pose3d.struct).publish();
        distPublisher = NetworkTableInstance.getDefault().getTable("Vision").getDoubleTopic("distance").publish();

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

    //  /**
    //  * check if the vision has a target using networktables
    //  * @return boolean - if the camera has a target
    //  */
    // public boolean hasTarget() {
    //     return hasTarget(CameraName.front);
    // }

    // /**
    //  * get the target's Y position in pixels
    //  * @param camera - the camera to check
    //  * @param offset - the offset to add to the value
    //  * @return double - the target's Y position in pixels
    //  */
    // public double getTY(CameraName camera, double offset) {
    //     if(isCameraInitialized(camera)) {
    //         switch(camera) {
    //             case front:
    //                 return camThread.getCameraObject().getCameraTable().getEntry("targetPixelsY").getDouble(0) + offset;

    //             default:
    //                 return 0;
    //         }
    //     } else {
    //         return 0;
    //     }
    // }

    // /**
    //  * get the target's X position in pixels
    //  * @param camera - the camera to check
    //  * @param offset - the offset to add to the value
    //  * @return double - the target's X position in pixels
    //  */
    // public double getTX(CameraName camera, double offset) {
    //     if(isCameraInitialized(camera)) {
    //         switch(camera) {
    //             case front:
    //                 return camThread.getCameraObject().getCameraTable().getEntry("targetPixelsX").getDouble(0) + offset;

    //             default:
    //                 return 0;
    //         }
    //     } else {
    //         return 0;
    //     }
    // }

    private boolean isCameraInitialized(CameraName camName) {
        return camThread.cameraInitialized;
    }

    private synchronized void updateVision() {
        Tuple<EstimatedRobotPose, Double> updates = camThread.getUpdates();

        // drivetrain.addVisionMeasurement(updates.k, updates.v);

        posePublisher.accept(updates.k.estimatedPose);
        pose = updates.k.estimatedPose;
    }


    private class CameraThread extends Thread {
        private EstimatedRobotPose pose = new EstimatedRobotPose(new Pose3d(), 0, List.of(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        private PhotonPoseEstimator poseEstimator;
        private PhotonCamera camera;
        private Double averageDistance = 0d;
        private CameraName camName;
        private Tuple<EstimatedRobotPose, Double> updates;
        private boolean hasTarget = false;
        private AprilTagFieldLayout tags;

        private final BooleanPublisher hasTargetPublisher;
        private final DoublePublisher targetsFoundPublisher;
        private final DoublePublisher timestampPublisher;
        private final DoublePublisher distancePublisher;
        private final StructPublisher<Pose3d> posePublisher;

        public boolean cameraInitialized = false;

        CameraThread(CameraName camName, Transform3d cameraPosition, AprilTagFieldLayout tagLayout) {
            this.camName = camName;

            initializeCamera();

            poseEstimator = new PhotonPoseEstimator(tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            tags = poseEstimator.getFieldTags();

            poseEstimator.setFieldTags(tags);

            // Initialize NetworkTables publishers
            hasTargetPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getBooleanTopic("hasTarget").publish();
            targetsFoundPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getDoubleTopic("targetsFound").publish();
            timestampPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getDoubleTopic("timestamp").publish();
            distancePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getDoubleTopic("distance").publish();
            posePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getStructTopic("pose", Pose3d.struct).publish();
        }

        @Override
        public void run() {
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
                                    poseEstimator.update(result).ifPresentOrElse(((pose) -> this.pose = pose), () -> {
                                        DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " pose not updated");
                                    });
                                } else {
                                    DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " pose ambiguity is high");
                                }

                                // grabs the distance to the best target (for the latest set of result)
                                totalDistances += result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();

                                hasTargetPublisher.set(true);
                                targetsFoundPublisher.set(numberOfResults);
                                timestampPublisher.set(result.getTimestampSeconds());
                                distancePublisher.set(totalDistances / numberOfResults);
                                posePublisher.set(pose.estimatedPose);
                            } else {
                                hasTargetPublisher.set(false);
                                targetsFoundPublisher.set(0);
                            }
                        }

                        // averages distance over all results
                        averageDistance = totalDistances / numberOfResults;
                        updates = new Tuple<EstimatedRobotPose, Double>(pose, averageDistance);
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
        public Tuple<EstimatedRobotPose, Double> getUpdates() {
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
