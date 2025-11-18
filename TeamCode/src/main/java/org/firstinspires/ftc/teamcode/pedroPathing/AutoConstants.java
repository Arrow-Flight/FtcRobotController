package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AutoConstants {
    // ===Limelight===
    public static Limelight3A limelight;
    public static Pose3D limelightPose;

    // ===Servos===
    public static Servo servoCamera;

    // ===Motors===
    public static DcMotor intake;
    public static DcMotor shooterLeft;
    public static DcMotor shooterRight;

    // ===Shooter PIDF===
    public static double shooterTargetPower;
    public static double previousError;
    public static final double shooterP = 0.001;
    public static final double shooterF = 0.00045;
    public static final double shooterD = 0.005;
    public static final int shooterTargetVelocity = 1200;

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

    public static class Blue {
        // ===Poses===
        public static final Pose preStart = new Pose(22.8, 128, Math.toRadians(-45));
        public static final Pose preFinal = new Pose(29.8, 119, Math.toRadians(-45));
        public static final Pose shootPose = new Pose(48, 96, Math.toRadians(-48));
        public static final Pose firstSpikeInitial = new Pose(48, 83.5, Math.toRadians(180));
        public static final Pose firstSpikeFinal = new Pose(15,83.5, Math.toRadians(180));
        public static final Pose secondSpikeInitial = new Pose(48,60, Math.toRadians(180));
        public static final Pose secondSpikeFinal = new Pose(15,60, Math.toRadians(180));
        public static final Pose thirdSpikeInitial = new Pose(48,35.5, Math.toRadians(180));
        public static final Pose thirdSpikeFinal = new Pose(15,35.5, Math.toRadians(180));
        public static final Pose endPose = new Pose(38,60, Math.toRadians(90));
    }

    public static class Red {
        // ===Poses===
        public static final Pose preStart = new Pose(121.2, 128, Math.toRadians(-135));
        public static final Pose preFinal = new Pose(114.2, 119, Math.toRadians(-135));
        public static final Pose shootPose = new Pose(96, 96, Math.toRadians(-132));
        public static final Pose firstSpikeInitial = new Pose(96, 83.5, Math.toRadians(0));
        public static final Pose firstSpikeFinal = new Pose(129,83.5, Math.toRadians(0));
        public static final Pose secondSpikeInitial = new Pose(96,60, Math.toRadians(0));
        public static final Pose secondSpikeFinal = new Pose(129,60, Math.toRadians(0));
        public static final Pose thirdSpikeInitial = new Pose(96,35.5, Math.toRadians(0));
        public static final Pose thirdSpikeFinal = new Pose(129,35.5, Math.toRadians(0));
        public static final Pose endPose = new Pose(106,60, Math.toRadians(90));
    }

    public static void Shoot(int State) {
        shooterRight.setPower(shooterTargetPower);
        shooterLeft.setPower(shooterTargetPower);

        if (pathTimer.getElapsedTimeSeconds() >= 5.0) {
            intake.setPower(0);
            shooterRight.setPower(0);
            shooterLeft.setPower(0);
            pathState = State;

        } else if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
            intake.setPower(1);
        }
    }
}

