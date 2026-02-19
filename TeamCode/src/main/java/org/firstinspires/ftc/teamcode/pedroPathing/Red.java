/*
package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Red.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.*;

import com.pedropathing.geometry.*;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@Autonomous(preselectTeleOp="MainOpRed")
public class Red extends OpMode {

    @Override
    public void init() {
        // Set Up servoCamera
        servoCamera = hardwareMap.get(Servo.class, "servoCamera");
        servoCamera.scaleRange(0.3, 1.0);
        servoCamera.setPosition(0.4);

        //Set Up intake
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set Up upper
        upper = hardwareMap.get(DcMotor.class, "upper");
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Up shooter motors
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(42.0, 0, 0, 18.5);

        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

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
        follower.update();

        if (timeout.getElapsedTimeSeconds() < 27) {
            // Step 1: Initial Move
            if (pathState == 0 && !follower.isBusy()) {
                currentPose = follower.getPose();

                goToShoot();

                pathTimer.resetTimer();
                pathState = 1;
            }

            // Step 2: Calculate Offsets and Fix Position
            else if (pathState ==1 && pathTimer.getElapsedTimeSeconds() > 2 && !follower.isBusy()) {
                currentPose = follower.getPose();
                xError = getStartingError().getX();
                yError = getStartingError().getY();

                Pose correctedTarget = new Pose(shootPose.getX() + xError, shootPose.getY() + yError, shootPose.getHeading());

                toShoot = new Path(new BezierLine(currentPose, correctedTarget));
                toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), correctedTarget.getHeading());
                follower.followPath(toShoot);
                pathState = 2;
            }

            // Step 3: Shoot Initial Balls
            else if (pathState == 2 && !follower.isBusy()) {
                Shoot(telemetry);
                pathState = 3;
                subState = 1;
            }

            // Step 4: Get Balls From First Spike
            else if (pathState == 3 && !follower.isBusy()) {
                currentPose = getCorrectedPose();

                collectSpike(firstSpikeInitial, firstSpikeFinal);
                pathState = 4;
            }

            // Step 5: Go to Shoot
            else if (pathState == 4 && !follower.isBusy()) {
                intake.setPower(0);

                goToShoot();
                pathState = 5;
            }

            // Step 6: Shoot First Spike Balls
            else if (pathState == 5 && !follower.isBusy()) {

                Shoot(telemetry);
                pathState = 6;
                subState = 1;
            }

            // Step 7: Get Balls From Second Spike
            else if (pathState == 6 && !follower.isBusy()) {
                currentPose = getCorrectedPose();

                collectSpike(secondSpikeInitial, secondSpikeFinal);
                pathState = 7;
            }

            // Step 8: Go to Shoot
            else if (pathState == 7 && !follower.isBusy()) {
                intake.setPower(0);

                goToShoot();
                pathState = 8;
            }

            // Step 9: Shoot Second Spike Balls
            else if (pathState == 8 && !follower.isBusy()) {

                Shoot(telemetry);
                pathState = 9;
                subState = 1;
            }

            // Step 10:  Get Balls From Third Spike
            else if (pathState == 7 && !follower.isBusy()) {
                currentPose = getCorrectedPose();

                collectSpike(thirdSpikeInitial, thirdSpikeFinal);
                pathState = 10;
            }

            // Step 11: Go to Shoot
            else if (pathState == 10 && !follower.isBusy()) {
                intake.setPower(0);

                goToShoot();
                pathState = 11;
            }

            // Step 12: Shoot Third Spike Balls
            else if (pathState == 11 && !follower.isBusy()) {

                Shoot(telemetry);
                pathState = 12;
                subState = 1;
            }
        } else {
            if (!timeoutTriggered && timeout.getElapsedTimeSeconds() >= 28) {
                timeoutTriggered = true;

                follower.breakFollowing();
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
                intake.setPower(0);
                upper.setPower(0);

                goToEnd = new Path(new BezierLine(shootPose, endPose));
                goToEnd.setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading());

                follower.followPath(goToEnd);
            }
        }
    }
}
 */