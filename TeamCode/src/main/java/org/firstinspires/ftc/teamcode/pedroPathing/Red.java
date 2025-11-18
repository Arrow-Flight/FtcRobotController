package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.endPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.firstSpikeFinal;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.firstSpikeInitial;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.preFinal;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.preStart;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.secondSpikeFinal;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.secondSpikeInitial;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.shootPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.thirdSpikeFinal;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.thirdSpikeInitial;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Shoot;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.firstSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.goToEnd;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.intake;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.limelight;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.limelightPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.pathState;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.pathTimer;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.preMove;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.previousError;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.secondSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.servoCamera;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shootFromFirstSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shootFromSecondSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shootFromThirdSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shootToFirstSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shootToSecondSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shootToThirdSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shooterD;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shooterF;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shooterLeft;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shooterP;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shooterRight;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shooterTargetVelocity;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.thirdSpike;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.shooterTargetPower;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Red extends OpMode {

    @Override
    public void init() {
        // Set Up servoCamera
        servoCamera = hardwareMap.get(Servo.class, "servoCamera");
        servoCamera.scaleRange(0.3, 1.0);
        servoCamera.setPosition(0.3);

        //Set Up intake
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Up shooter motors
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Up Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7); // use your AprilTag pipeline

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(preStart);

        // Build Paths
        preMove = new Path(new BezierLine(preStart, preFinal));
        preMove.setLinearHeadingInterpolation(preStart.getHeading(), preFinal.getHeading());

        shootToFirstSpike = new Path(new BezierLine(shootPose, firstSpikeInitial));
        shootToFirstSpike.setLinearHeadingInterpolation(shootPose.getHeading(),firstSpikeInitial.getHeading());

        firstSpike = new Path(new BezierLine(firstSpikeInitial, firstSpikeFinal));
        firstSpike.setLinearHeadingInterpolation(firstSpikeInitial.getHeading(),firstSpikeFinal.getHeading());

        shootFromFirstSpike = new Path(new BezierLine(firstSpikeInitial, shootPose));
        shootFromFirstSpike.setLinearHeadingInterpolation(firstSpikeFinal.getHeading(),shootPose.getHeading());

        shootToSecondSpike = new Path(new BezierLine(shootPose,secondSpikeInitial));
        shootToSecondSpike.setLinearHeadingInterpolation(shootPose.getHeading(),secondSpikeInitial.getHeading());

        secondSpike = new Path(new BezierLine(secondSpikeInitial,secondSpikeFinal));
        secondSpike.setLinearHeadingInterpolation(secondSpikeInitial.getHeading(),secondSpikeFinal.getHeading());

        shootFromSecondSpike = new Path(new BezierLine(secondSpikeFinal, shootPose));
        shootFromSecondSpike.setLinearHeadingInterpolation(secondSpikeFinal.getHeading(), shootPose.getHeading());

        shootToThirdSpike = new Path(new BezierLine(shootPose,thirdSpikeInitial));
        shootToThirdSpike.setLinearHeadingInterpolation(shootPose.getHeading(),thirdSpikeInitial.getHeading());

        thirdSpike = new Path(new BezierLine(thirdSpikeInitial,thirdSpikeFinal));
        thirdSpike.setLinearHeadingInterpolation(thirdSpikeInitial.getHeading(),thirdSpikeFinal.getHeading());

        shootFromThirdSpike = new Path(new BezierLine(thirdSpikeFinal, shootPose));
        shootFromThirdSpike.setLinearHeadingInterpolation(thirdSpikeFinal.getHeading(), shootPose.getHeading());

        goToEnd = new Path(new BezierLine(shootPose, endPose));
        goToEnd.setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading());

        // Add Timer
        pathTimer = new Timer();

        // Initial Telemetry
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
        // Shooter PIDF
        double currentError = (shooterTargetVelocity - ((DcMotorEx)shooterRight).getVelocity());
        shooterTargetPower = ((shooterF * shooterTargetVelocity) + (shooterP * (shooterTargetVelocity - ((DcMotorEx)shooterRight).getVelocity())) + (shooterD * (currentError - previousError)));
        telemetry.addData("Name", pathTimer.getElapsedTimeSeconds());
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

                Pose startPose = new Pose(xInches, yInches, headingRadians);
                moveToShoot = new Path(new BezierLine(startPose, shootPose));
                moveToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

                follower.followPath(moveToShoot);
                limelight.stop();
                servoCamera.setPosition(1.0);

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
                pathTimer.resetTimer();
                pathState = 2;
            }
        }

        // Step 3: Shoot Ball
        else if (pathState == 2 && !follower.isBusy()) {
            Shoot(3);
        }
        // Step 4: Move to Get Balls From First Spike
        else if (pathState == 3 && !follower.isBusy()) {
            follower.followPath(shootToFirstSpike);
            pathState = 4;

        }
        // Step 5: Intake Balls On First Spike
        else if (pathState == 4 && !follower.isBusy()) {
            intake.setPower(1);
            follower.followPath(firstSpike);
            pathState = 5;

        }
        // Step 5: Return to Shoot Pose
        else if (pathState == 5 && !follower.isBusy()) {
            intake.setPower(0);
            follower.followPath(shootFromFirstSpike);
            pathTimer.resetTimer();
            pathState = 6;

        }
        // Step 6: Shoot Balls
        else if (pathState == 6 && !follower.isBusy()) {
            Shoot(7);
        }
        // Step 7: Move to Get Balls From Second Spike
        else if (pathState == 7 && !follower.isBusy()) {
            follower.followPath(shootToSecondSpike);
            pathState = 8;
        }
        // Step 8: Intake Balls on Second Spike
        else if (pathState == 8 && !follower.isBusy()) {
            intake.setPower(1);
            follower.followPath(secondSpike);
            pathState = 9;
        }
        // Step 9: Return to Shoot Pose
        else if (pathState == 9 && !follower.isBusy()) {
            intake.setPower(0);
            follower.followPath(shootFromSecondSpike);
            pathTimer.resetTimer();
            pathState = 10;
        }
        // Step 10: Shoot Balls
        else if (pathState == 10 && !follower.isBusy()) {
            Shoot(11);
        }
        // Step 11: Move to Get Balls From Third Spike
        else if (pathState == 11 && !follower.isBusy()) {
            follower.followPath(shootToThirdSpike);
            pathState = 12;
        }
        // Step 12: Intake Balls on Third Spike
        else if (pathState == 12 && !follower.isBusy()) {
            intake.setPower(1);
            follower.followPath(thirdSpike);
            pathState = 13;
        }
        // Step 13: Return to Shoot Pose
        else if (pathState == 13 && !follower.isBusy()) {
            intake.setPower(0);
            follower.followPath(shootFromThirdSpike);
            pathTimer.resetTimer();
            pathState = 14;
        }
        // Step 14: Shoot Balls
        else if (pathState == 14 && !follower.isBusy()) {
            Shoot(15);
        }
        // Step 15: Go To End
        else if (pathState == 15 && !follower.isBusy()) {
            follower.followPath(goToEnd);
            pathState = 16;
        }
        previousError = currentError;
    }
}