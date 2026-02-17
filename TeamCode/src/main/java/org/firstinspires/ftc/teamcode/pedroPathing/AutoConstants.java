package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AutoConstants {
    // ===Limelight===
    public static Limelight3A limelight;

    // ===Servos===
    public static Servo servoCamera;

    // ===Motors===
    public static DcMotor intake;
    public static DcMotor upper;
    public static DcMotorEx shooterLeft;
    public static DcMotorEx shooterRight;

    // ===Paths===
    public static Path toShoot;
    public static Path shootToFirstSpike;
    public static Path firstSpike;
    public static Path shootFromFirstSpike;
    public static Path shootToSecondSpike;
    public static Path secondSpike;
    public static Path shootFromSecondSpike;
    public static Path shootToThirdSpike;
    public static Path thirdSpike;
    public static Path shootFromThirdSpike;
    public static Path goToEnd;

    // ===Misc.===
    public static Follower follower;
    public static Timer pathTimer;
    public static Timer timeout;
    public static int pathState;
    public static int currentState = 0;
    public static int ballsShot = 0;
    public static int shooterTargetVelocity = 1200;
    public static double vel;
    public static Pose currentPose;
    public static int subState;
    public static boolean timeoutTriggered = false;

    public static class Blue {
        // ===Poses===
        public static Pose startingPose = new Pose(23, 128, Math.toRadians(-36));
        public static Pose shootPose = new Pose(56, 92, Math.toRadians(-36));
        public static Pose firstSpikeInitial = new Pose(40, 85, Math.toRadians(180));
        public static Pose firstSpikeFinal = new Pose(15,85, Math.toRadians(180));
        public static Pose secondSpikeInitial = new Pose(40,62, Math.toRadians(180));
        public static Pose secondSpikeFinal = new Pose(5,62, Math.toRadians(180));
        public static Pose thirdSpikeInitial = new Pose(40,40, Math.toRadians(180));
        public static Pose thirdSpikeFinal = new Pose(5,40, Math.toRadians(180));
        public static Pose endPose = new Pose(35,75, Math.toRadians(0));

        public static int pipeline = 7;
    }
    public static class Red {
        // ===Poses===
        public static Pose startingPose = new Pose(121.2, 128, Math.toRadians(-135));
        public static Pose shootPose = new Pose(92, 92, Math.toRadians(-138));
        public static Pose firstSpikeInitial = new Pose(100, 85, Math.toRadians(5));
        public static Pose firstSpikeFinal = new Pose(129,85, Math.toRadians(5));
        public static Pose secondSpikeInitial = new Pose(100,62, Math.toRadians(5));
        public static Pose secondSpikeFinal = new Pose(139,62, Math.toRadians(5));
        public static Pose thirdSpikeInitial = new Pose(100,40, Math.toRadians(5));
        public static Pose thirdSpikeFinal = new Pose(139,40, Math.toRadians(5));
        public static Pose endPose = new Pose(115,75, Math.toRadians(180));

        public static int pipeline = 8;
    }

    // =======================
    // SHOOT STATE MACHINE
    // =======================
    public static void Shoot(int State, int shots, Pose shootAt, Telemetry telemetry) {

        // Step 1: Move from current pose to shoot pose
        if (subState == 1 && !follower.isBusy()) {
            currentPose = follower.getPose();

            toShoot = new Path(new BezierLine(currentPose, shootAt));
            toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), shootAt.getHeading());

            follower.followPath(toShoot);
            pathTimer.resetTimer();
            subState = 2;
        }

        // 2) Relocalize with Limelight (once, when settled)
        else if (subState == 2 && !follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose();
                double rawX = botPose.getPosition().x;
                double rawY = botPose.getPosition().y;

                double xInches = rawX * 39.37;
                double yInches = rawY * 39.37;

                double xCorrect = xInches + 72;
                double yCorrect = -yInches + 72;

                telemetry.addData("x", xCorrect);
                telemetry.addData("y", yCorrect);
                telemetry.addData("LLHeading", botPose.getOrientation().getYaw());

                // Recalculate path to shoot pose with updated position
                follower.setPose(new Pose(xCorrect, yCorrect, follower.getHeading()));
                currentPose = follower.getPose();
                toShoot = new Path(new BezierLine(currentPose, shootAt));
                toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), shootAt.getHeading());
                follower.followPath(toShoot);
                pathTimer.resetTimer();
            }
            subState = 4;
        }


        // 3) Shoot balls
        else if (subState == 3 && !follower.isBusy()) {
            if (ballsShot == shots) {
                pathTimer.resetTimer();
                pathState = State;

                upper.setPower(0);
                intake.setPower(0);
                shooterRight.setVelocity(0);
                shooterLeft.setVelocity(0);

                ballsShot = 0;
                subState = 0;
                return;
            }

            shooterLeft.setVelocity(shooterTargetVelocity);
            shooterRight.setVelocity(shooterTargetVelocity);

            if (vel >= 1200 && currentState == 0) {
                currentState = 1;
            }
            else if (currentState == 1) {
                upper.setPower(1);
                intake.setPower(1);

                if (vel <= 1100) {
                    upper.setPower(0);
                    intake.setPower(0);
                    ballsShot++;
                    currentState = 0;
                }
            }
        }
    }
}

