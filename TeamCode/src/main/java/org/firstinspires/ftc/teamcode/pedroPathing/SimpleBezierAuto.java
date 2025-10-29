package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Simple BezierLine Auto", group = "Examples")
public class SimpleBezierAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Define positions
    private final Pose startPose = new Pose(34.000, 135.500, Math.toRadians(-90));
    private final Pose Line1 = new Pose(48, 80, Math.toRadians(180));
    private final Pose Line2 = new Pose(25, 80, Math.toRadians(180));
    private final Pose endPose = new Pose(48, 95.500, Math.toRadians(-45));
    private Path Path1;
    private Path Path2;
    private Path Path3;
    private Path Path4;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build a simple path between the two poses
        Path1 = new Path(new BezierLine(startPose, endPose));
        Path1.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        Path2 = new Path(new BezierLine(endPose, Line1));
        Path2.setLinearHeadingInterpolation(endPose.getHeading(), Line1.getHeading());

        Path3 = new Path(new BezierLine(Line1, Line2));
        Path3.setTangentHeadingInterpolation();

        Path4 = new Path(new BezierLine(Line2, endPose));
        Path4.setLinearHeadingInterpolation(Line2.getHeading(), endPose.getHeading());

        pathTimer = new Timer();
        pathState = 0;

        telemetry.addLine("Initialized and ready");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        follower.followPath(Path1);
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            if (pathState == 0) {
                // Finished first move: start pause timer
                pathTimer.resetTimer();
                pathState = 1;
            } else if (pathState == 1) {
                // Check both time and position before continuing
                double dx = follower.getPose().getX() - endPose.getX();
                double dy = follower.getPose().getY() - endPose.getY();
                double distance = Math.hypot(dx, dy);

                if (pathTimer.getElapsedTimeSeconds() >= 5.0 && distance < 1.0) {
                    follower.followPath(Path2);
                    pathState = 2;
                }
            } else if (pathState == 2) {
                // Finished path2 (end -> Line1)
                double dx = follower.getPose().getX() - Line1.getX();
                double dy = follower.getPose().getY() - Line1.getY();
                if (Math.hypot(dx, dy) < 1.0) {
                    follower.followPath(Path3);
                    pathState = 3;
                }
            } else if (pathState == 3) {
                // Finished path3 (Line1 -> Line2)
                double dx = follower.getPose().getX() - Line2.getX();
                double dy = follower.getPose().getY() - Line2.getY();
                if (Math.hypot(dx, dy) < 1.0) {
                    follower.followPath(Path4);
                    pathState = 4;
                }
            } else if (pathState == 4) {
                // Finished returning to endPose
                double dx = follower.getPose().getX() - endPose.getX();
                double dy = follower.getPose().getY() - endPose.getY();
                if (Math.hypot(dx, dy) < 1.0) {
                    pathState = 5; // Done
                    telemetry.addLine("All paths complete and robot in position");
                }
            }
        }

        telemetry.addData("segment", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading_deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}