package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;
import com.bylazar.telemetry.PanelsTelemetry;
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

    @Configurable
    public static class Blue {
        // ===Poses===
        public static Pose preStart = new Pose(22.8, 128, Math.toRadians(-45));
        public static Pose preFinal = new Pose(29.8, 119, Math.toRadians(-45));
        public static Pose shootPose = new Pose(48, 96, Math.toRadians(-48));
        public static Pose firstSpikeInitial = new Pose(40, 83, Math.toRadians(177));
        public static Pose firstSpikeFinal = new Pose(15,83.5, Math.toRadians(177));
        public static Pose secondSpikeInitial = new Pose(40,60, Math.toRadians(177));
        public static Pose secondSpikeFinal = new Pose(15,60, Math.toRadians(177));
        public static Pose thirdSpikeInitial = new Pose(40,35.5, Math.toRadians(177));
        public static Pose thirdSpikeFinal = new Pose(15,35.5, Math.toRadians(177));
        public static Pose endPose = new Pose(38,60, Math.toRadians(90));
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

    // Internal state machine for shooter control
    private static int shootStage = 0;
    private static int shotsRemaining = 0;

    private static double lastVelocity = 0;
    private static boolean shotDetected = false;
    private static final Timer shootTimer = new Timer();
    public static void Shoot(int State, int Shots) {
        telemetry.addData("Shots", shotsRemaining);
        telemetry.update();
        double velocity = shooterRight.getVelocity();


        switch (shootStage) {
            // Stage 0: Initialize sequence
            case 0:
                shotsRemaining = Shots;
                shooterLeft.setPower(shooterTargetPower);
                shooterRight.setPower(shooterTargetPower);
                shotDetected = false;
                shootStage = 1;
                break;

            // Stage 1: Wait for shooter to spin up
            case 1:
                if (velocity >= shooterTargetVelocity * 0.95) {
                    intake.setPower(1.0);
                    shootStage = 2;
                }
                break;

            // Stage 2: Detect a shot by velocity drop
            case 2:
                if (velocity < lastVelocity * 0.82) {
                    // Shot detected
                    intake.setPower(0);
                    shotDetected = true;
                    shootTimer.resetTimer();
                    shootStage = 3;
                }
                break;

            // Stage 3: Re-spin
            case 3:
                if (velocity >= shooterTargetVelocity * 0.96) {

                    shotsRemaining--;

                    if (shotsRemaining > 0) {
                        intake.setPower(1.0);
                        shootStage = 2;
                    } else {
                        shootStage = 4;
                    }
                }
                break;

            // Stage 4: Shutdown and return control
            case 4:
                intake.setPower(0);
                shooterLeft.setPower(0);
                shooterRight.setPower(0);

                // Reset internal state
                shootStage = 0;
                shotDetected = false;

                // Move to next state in OpMode
                pathState = State;
                break;
        }

        lastVelocity = velocity;
    }

}

