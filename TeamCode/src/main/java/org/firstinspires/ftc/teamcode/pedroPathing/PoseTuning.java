package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Blue.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@Config
@TeleOp
public class PoseTuning extends LinearOpMode {
    public static double poseX = 72;
    public static double poseY = 72;
    public static double poseHeading = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline); // use your AprilTag pipeline
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        pathState = 0;
        pathTimer = new Timer();
        pathTimer.resetTimer();

        waitForStart();
        while (opModeIsActive()) {
            follower.update();

            if (pathState == 0 && !follower.isBusy()) {
                currentPose = follower.getPose();

                toShoot = new Path(new BezierLine(currentPose, shootPose));
                toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), shootPose.getHeading());

                follower.followPath(toShoot);

                pathTimer.resetTimer();
                pathState = 1;
            }

            // Step 2: Calculate Offsets and Fix Position
            else if (pathState == 1 && pathTimer.getElapsedTimeSeconds() > 2 && !follower.isBusy()) {
                xError = getStartingError().getX();
                yError = getStartingError().getY();

                currentPose = getCorrectedPose();

                toShoot = new Path(new BezierLine(currentPose, shootPose));
                toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), shootPose.getHeading());
                follower.followPath(toShoot);
                pathTimer.resetTimer();
                shots = 0;
                spunUp = false;
                shootState = 1;
                pathState = 2;
            }
            if (pathState == 2) {
                follower.holdPoint(new Pose(poseX, poseY, Math.toRadians(poseHeading)));
            }
        }
    }
}