package org.firstinspires.ftc.teamcode.pedroPathing;

//import com.acmerobotics.dashboard.config.Config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
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
    public static int pathState;
    public static int currentState = 0;
    public static int ballsShot = 0;
    public static int shooterTargetVelocity = 1200;
    public static double vel;
    public static Pose currentPose;
    public static int subState;
    static Pose3D botPose;

    public static class Blue {
        // ===Poses===
        public static Pose startingPose = new Pose(22.8, 128, Math.toRadians(-45));
        public static Pose shootPose = new Pose(48, 102, Math.toRadians(-50));
        public static Pose firstSpikeInitial = new Pose(40, 83, Math.toRadians(180));
        public static Pose firstSpikeFinal = new Pose(15,83.5, Math.toRadians(180));
        public static Pose secondSpikeInitial = new Pose(40,60, Math.toRadians(180));
        public static Pose secondSpikeFinal = new Pose(5,60, Math.toRadians(180));
        public static Pose thirdSpikeInitial = new Pose(40,37, Math.toRadians(180));
        public static Pose thirdSpikeFinal = new Pose(5,37, Math.toRadians(180));
        public static Pose endPose = new Pose(38,60, Math.toRadians(90));

        public static int pipeline = 7;
    }
    public static class Red {
        // ===Poses===
        public static Pose preStart = new Pose(121.2, 128, Math.toRadians(-135));
        public static Pose shootPose = new Pose(96, 96, Math.toRadians(-135));
        public static Pose firstSpikeInitial = new Pose(104, 83, Math.toRadians(5));
        public static Pose firstSpikeFinal = new Pose(129,83.5, Math.toRadians(5));
        public static Pose secondSpikeInitial = new Pose(104,60, Math.toRadians(5));
        public static Pose secondSpikeFinal = new Pose(139,60, Math.toRadians(5));
        public static Pose thirdSpikeInitial = new Pose(104,37, Math.toRadians(5));
        public static Pose thirdSpikeFinal = new Pose(139,37, Math.toRadians(5));
        public static Pose endPose = new Pose(106,60, Math.toRadians(-90));

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
            subState = 2;
        }

        // 2) Relocalize with Limelight (once, when settled)
        else if (subState == 2 && !follower.isBusy()) {

            Pose llPose = getPoseFromLimelight();
            if (llPose != null) {
                follower.setPose(llPose);
                telemetry.addLine("Used April Tag!");
                telemetry.update();
            }

            subState = 3;
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
    private static Pose getPoseFromLimelight() {

        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D botPose = result.getBotpose();
        if (botPose == null) return null;

        // FTC field coordinates (meters)
        double xMeters = botPose.getPosition().x;
        double yMeters = botPose.getPosition().y;
        double headingRad = botPose.getOrientation().getYaw();

        // Convert meters → inches
        double xInches = xMeters * 39.3701;
        double yInches = yMeters * 39.3701;

        // Convert FTC → Pedro coordinates
        return new Pose(xInches, yInches, headingRad, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }



}

