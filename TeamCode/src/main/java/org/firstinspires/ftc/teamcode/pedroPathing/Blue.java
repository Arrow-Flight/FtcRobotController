package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
@Autonomous
public class Blue extends OpMode {

    // Limelight
    private Limelight3A limelight;
    private Pose3D limelightPose;

    // Follower and Timer
    private Follower follower;
    private Timer pathTimer;

    // Define positions
    private final Pose preStart = new Pose(22.8, 128, Math.toRadians(-45));
    private final Pose preFinal = new Pose(29.8, 119, Math.toRadians(-45));
    private final Pose shootPose = new Pose(48, 96, Math.toRadians(-45));

    // Define Paths
    private Path preMove;

    int pathState = 0;

    @Override
    public void init() {
        Servo servoCamera = hardwareMap.get(Servo.class, "servoCamera");
        servoCamera.scaleRange(0.3, 1.0);
        servoCamera.setPosition(0.3);

        // Set Up Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7); // use your AprilTag pipeline

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(preStart);

        // Build a simple path between the two poses
        preMove = new Path(new BezierLine(preStart, preFinal));
        preMove.setLinearHeadingInterpolation(preStart.getHeading(), preFinal.getHeading());

        pathTimer = new Timer();

        telemetry.addLine("Initialized and ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        telemetry.update();
        follower.update();

        // Step 1: Run the first move
        if (pathState == 0 && !follower.isBusy()) {
            follower.followPath(preMove);
            pathTimer.resetTimer(); // track how long the move runs
            pathState = 1;
        }

        // Step 2: Wait for Limelight pose (with retry + timeout)
        else if (pathState == 1) {
            // Try to get a valid Limelight pose
            if (limelight.isRunning()) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    limelightPose = result.getBotpose();
                }
            }

            // If we got a valid pose
            Path moveToShoot;
            if (limelightPose != null) {
                double xInches = -10 - (limelightPose.getPosition().y * 39.37);
                double yInches = 159.7 + (limelightPose.getPosition().x * 39.37);
                double headingRadians = Math.toRadians(limelightPose.getOrientation().getYaw() - 90);
                telemetry.addData("X", xInches);
                telemetry.addData("Y", yInches);
                telemetry.addData("Heading", headingRadians);
/*
                Pose startPose = new Pose(xInches, yInches, headingRadians);
                moveToShoot = new Path(new BezierLine(startPose, shootPose));
                moveToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

                follower.followPath(moveToShoot);
                limelight.stop();
 */
                pathState = 2;
            }

            // If no valid pose after 2 seconds, fall back
            else if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                telemetry.addLine("No Limelight pose — using estimated position");
                telemetry.update();

                Pose fallbackPose = follower.getPose(); // use current follower pose
                moveToShoot = new Path(new BezierLine(fallbackPose, shootPose));
                moveToShoot.setLinearHeadingInterpolation(fallbackPose.getHeading(), shootPose.getHeading());

                follower.followPath(moveToShoot);
                limelight.stop();
                pathState = 2;
            }
        }

        // Step 3: After moving to shoot pose
        else if (pathState == 2 && !follower.isBusy()) {
            telemetry.addLine("Arrived at shoot pose!");
            telemetry.update();
            // Ready for next action (like shooting)
        }
    }

}
