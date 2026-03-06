package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Blue.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@Autonomous(preselectTeleOp="MainOpBlue")
public class Blue extends OpMode {

    @Override
    public void init() {
        // Set Up servoCamera
        servoCamera = hardwareMap.get(Servo.class, "servoCamera");
        servoCamera.scaleRange(0.3, 1.0);
        servoCamera.setPosition(0.4);

        //Set Up intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set Up upper
        upper = hardwareMap.get(DcMotor.class, "upper");
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Up shooter motors
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        pidfController = new PIDFController(new com.pedropathing.control.PIDFCoefficients(0.002, 0, 0, 0.76));
        pidfController.setTargetPosition(2800);


        // Set Up Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline); // use your AprilTag pipeline

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        pathState = 0;

        // Add Timers
        pathTimer = new Timer();
        timeout = new Timer();
    }

    @Override
    public void start() {
        limelight.start();
        pathTimer.resetTimer();
        timeout.resetTimer();
    }

    @Override
    public void loop() {
        vel = -getVelocity();
        /*
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("vel", vel);
        dashboardTelemetry.update();
         */
        telemetry.addData("vel", vel);
        telemetry.update();
        follower.update();

        if (timeout.getElapsedTimeSeconds() < 27) {
            // Step 1: Initial Move
            if (pathState == 0 && !follower.isBusy()) {
                currentPose = follower.getPose();

                toShoot = new Path(new BezierLine(currentPose, shootPose));
                toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), shootPose.getHeading());

                follower.followPath(toShoot);

                pathTimer.resetTimer();
                pathState = 1;
            }

            // Step 2: Calculate Offsets and Fix Position
            else if (pathState ==1 && pathTimer.getElapsedTimeSeconds() > 2 && !follower.isBusy()) {
                currentPose = follower.getPose();
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

            // Step 3: Shoot Initial Balls
            else if (pathState == 2 && !follower.isBusy()) {
                Shoot(3);
                subState = 1;
            }

            // Step 4: Get Balls From First Spike
            else if (pathState == 3 && !follower.isBusy()) {
                currentPose = getCorrectedPose();

                collectSpike(4,firstSpikeInitial, firstSpikeFinal);
            }

            // Step 5: Go to Shoot
            else if (pathState == 4 && !follower.isBusy()) {
                currentPose = getCorrectedPose();
                intake.setPower(0);

                goToShoot(shootPose);
                pathTimer.resetTimer();
                shots = 0;
                shootState = 1;
                pathState = 5;
            }

            // Step 6: Shoot First Spike Balls
            else if (pathState == 5 && !follower.isBusy()) {

                Shoot(6);
                subState = 1;
            }

            // Step 7: Get Balls From Second Spike
            else if (pathState == 6 && !follower.isBusy()) {
                currentPose = getCorrectedPose();

                collectSpike(7,secondSpikeInitial, secondSpikeFinal);
            }

            // Step 8: Go to Shoot
            else if (pathState == 7 && !follower.isBusy()) {
                currentPose = getCorrectedPose();
                intake.setPower(0);

                goToShoot(shootPose);
                pathTimer.resetTimer();
                shots = 0;
                shootState = 1;
                pathState = 8;
            }

            // Step 9: Shoot Second Spike Balls
            else if (pathState == 8 && !follower.isBusy()) {

                Shoot(9);
                subState = 1;
            }

            // Step 10:  Get Balls From Third Spike
            else if (pathState == 9 && !follower.isBusy()) {
                currentPose = getCorrectedPose();

                collectSpike(10, thirdSpikeInitial, thirdSpikeFinal);
            }

            // Step 11: Go to Shoot
            else if (pathState == 10 && !follower.isBusy()) {
                currentPose = getCorrectedPose();
                intake.setPower(0);

                goToShoot(shootPose);
                pathTimer.resetTimer();
                shots = 0;
                shootState = 1;
                pathState = 11;
            }

            // Step 12: Shoot Third Spike Balls
            else if (pathState == 11 && !follower.isBusy()) {

                Shoot(12);
                subState = 1;
            }
        } else {
            if (!timeoutTriggered && timeout.getElapsedTimeSeconds() >= 28) {
                currentPose = getCorrectedPose();
                timeoutTriggered = true;

                follower.breakFollowing();
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
                intake.setPower(0);
                upper.setPower(0);

                goToEnd = new Path(new BezierLine(currentPose, endPose));
                goToEnd.setLinearHeadingInterpolation(currentPose.getHeading(),endPose.getHeading());

                follower.followPath(goToEnd);
            }
        }
    }
}