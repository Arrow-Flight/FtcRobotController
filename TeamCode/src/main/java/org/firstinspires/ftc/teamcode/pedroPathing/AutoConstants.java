package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

public class AutoConstants {
    // ===Limelight===
    public static Limelight3A limelight;
    public static Pose3D limelightPose;

    // ===Servos===
    public static Servo servoCamera;

    // ===Motors===
    public static DcMotor intake;
    public static DcMotor upper;
    public static DcMotorEx shooterLeft;
    public static DcMotorEx shooterRight;

    // ===Paths===
    public static Path move;

    // ===Misc===
    public static Follower follower;
    public static Timer pathTimer;
    public static double shooterP = 42;
    public static double shooterF = 13.5329;
    public static int shooterTargetVelocity = 1500;
    static double curVelocity;


    public static class Blue {
        // ===Poses===
        public static Pose preStart = new Pose(22.8, 128, Math.toRadians(-45));
        public static Pose preFinal = new Pose(29.8, 119, Math.toRadians(-45));
        public static Pose shootPose = new Pose(48, 96, Math.toRadians(-48));
        public static Pose firstSpikeInitial = new Pose(45, 83, Math.toRadians(177));
        public static Pose firstSpikeFinal = new Pose(15,83.5, Math.toRadians(177));
        public static Pose secondSpikeInitial = new Pose(45,60, Math.toRadians(177));
        public static Pose secondSpikeFinal = new Pose(5,60, Math.toRadians(177));
        public static Pose thirdSpikeInitial = new Pose(45,37, Math.toRadians(177));
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

    public static void init(HardwareMap hw) {
        shooterLeft = hw.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hw.get(DcMotorEx.class, "shooterRight");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(42.0, 0, 0, 13.5329);
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        intake = hw.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        upper = hw.get(DcMotorEx.class, "upper");
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setDirection(DcMotorSimple.Direction.REVERSE);
        upper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hw);
    }

    public static void Shoot(Pose current, Pose shoot) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, 0, 0, shooterF);
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        move = new Path(new BezierLine(current, shoot));
        move.setLinearHeadingInterpolation(current.getHeading(), shoot.getHeading());
        follower.followPath(move);

        shooterLeft.setVelocity(shooterTargetVelocity);
        shooterRight.setVelocity(shooterTargetVelocity);
        intake.setPower(0.3);

        curVelocity = (shooterRight.getVelocity() + shooterLeft.getVelocity())/2;
    }
}

