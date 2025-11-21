package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.ShooterPIDF.shooterTargetPower;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.ShooterPIDF.vel;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AutoConstants {
    // ===Limelight===
    public static Limelight3A limelight;
    public static Pose3D limelightPose;

    // ===Servos===
    public static Servo servoCamera;

    // ===Motors===
    public static DcMotor intake;
    public static DcMotorEx shooterLeft;
    public static DcMotorEx shooterRight;

    // ===Paths===
    public static Path preMove;
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

    @Config
    public static class ShooterPIDF {
        public static double shooterTargetPower;
        public static double previousError;
        public static double shooterP = 0.001;
        public static double shooterF = 0.00045;
        public static double shooterD = 0.005;
        public static int shooterTargetVelocity = 1200;
        public static double vel;
    }
    public static class Blue {
        // ===Poses===
        public static Pose preStart = new Pose(22.8, 128, Math.toRadians(-45));
        public static Pose preFinal = new Pose(29.8, 119, Math.toRadians(-45));
        public static Pose shootPose = new Pose(48, 96, Math.toRadians(-48));
        public static Pose firstSpikeInitial = new Pose(40, 83, Math.toRadians(177));
        public static Pose firstSpikeFinal = new Pose(15,83.5, Math.toRadians(177));
        public static Pose secondSpikeInitial = new Pose(40,60, Math.toRadians(177));
        public static Pose secondSpikeFinal = new Pose(5,60, Math.toRadians(177));
        public static Pose thirdSpikeInitial = new Pose(40,37, Math.toRadians(177));
        public static Pose thirdSpikeFinal = new Pose(5,37, Math.toRadians(177));
        public static Pose endPose = new Pose(38,60, Math.toRadians(90));
    }
    public static class Red {
        // ===Poses===
        public static Pose preStart = new Pose(121.2, 128, Math.toRadians(-135));
        public static Pose preFinal = new Pose(114.2, 119, Math.toRadians(-135));
        public static Pose shootPose = new Pose(96, 96, Math.toRadians(-132));
        public static Pose firstSpikeInitial = new Pose(96, 83.5, Math.toRadians(0));
        public static Pose firstSpikeFinal = new Pose(129,83.5, Math.toRadians(0));
        public static Pose secondSpikeInitial = new Pose(96,60, Math.toRadians(0));
        public static Pose secondSpikeFinal = new Pose(129,60, Math.toRadians(0));
        public static Pose thirdSpikeInitial = new Pose(96,35.5, Math.toRadians(0));
        public static Pose thirdSpikeFinal = new Pose(129,35.5, Math.toRadians(0));
        public static Pose endPose = new Pose(106,60, Math.toRadians(90));
    }

    public static void Shoot(int State, int shots) {
        shooterLeft.setPower(ShooterPIDF.shooterTargetPower);
        shooterRight.setPower(ShooterPIDF.shooterTargetPower);
        vel = (shooterLeft.getVelocity() + shooterRight.getVelocity())/2;
        if (ballsShot == shots) {
            pathState = State;
            intake.setPower(0);
            shooterRight.setPower(0);
            shooterLeft.setPower(0);
            currentState = 0;
            ballsShot = 0;
        } else {
            if (ballsShot == 1) {
                shooterTargetPower = 1250;
                if (vel >= 1250 && currentState == 0) {
                    currentState = 1;
                } else if (currentState == 1) {
                    intake.setPower(1);
                    if (vel <= 1100) {
                        intake.setPower(0);
                        ballsShot++;
                        currentState = 0;
                    }
                }
            } else {
                shooterTargetPower = 1200;
                if (vel >= 1200 && currentState == 0) {
                    currentState = 1;
                } else if (currentState == 1) {
                    intake.setPower(1);
                    if (vel <= 1100) {
                        intake.setPower(0);
                        ballsShot++;
                        currentState = 0;
                    }
                }
            }
        }
    }

}

