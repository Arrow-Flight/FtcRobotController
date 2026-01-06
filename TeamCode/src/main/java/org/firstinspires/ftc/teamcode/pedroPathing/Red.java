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

@Autonomous
public class Red extends OpMode {

    private enum AutoState {
        PRE_MOVE,
        LIMELIGHT_ALIGN,
        SHOOT_INITIAL,

        MOVE_TO_SPIKE,
        INTAKE_SPIKE,
        RETURN_FROM_SPIKE,
        SHOOT_SPIKE,

        GO_TO_END,
        DONE
    }

    private AutoState state = AutoState.PRE_MOVE;
    private int currentSpike = 0;

    private Path[] moveToSpike;
    private Path[] intakeSpike;
    private Path[] returnFromSpike;

    private final int[] shootStateIds = {7, 11, 15};

    @Override
    public void init() {
        // Camera
        servoCamera = hardwareMap.get(Servo.class, "servoCamera");
        servoCamera.scaleRange(0.3, 1.0);
        servoCamera.setPosition(0.3);

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(preStart);

        buildPaths();
        buildSpikeArrays();

        pathTimer = new Timer();

        telemetry.addLine("Initialized and ready");
        telemetry.update();
    }

    private void buildPaths() {
        preMove = new Path(new BezierLine(preStart, preFinal));
        preMove.setLinearHeadingInterpolation(preStart.getHeading(), preFinal.getHeading());

        shootToFirstSpike = new Path(new BezierLine(shootPose, firstSpikeInitial));
        shootToFirstSpike.setLinearHeadingInterpolation(shootPose.getHeading(), firstSpikeInitial.getHeading());

        firstSpike = new Path(new BezierLine(firstSpikeInitial, firstSpikeFinal));
        firstSpike.setLinearHeadingInterpolation(firstSpikeInitial.getHeading(), firstSpikeFinal.getHeading());

        shootFromFirstSpike = new Path(new BezierLine(firstSpikeInitial, shootPose));
        shootFromFirstSpike.setLinearHeadingInterpolation(firstSpikeFinal.getHeading(), shootPose.getHeading());

        shootToSecondSpike = new Path(new BezierLine(shootPose, secondSpikeInitial));
        shootToSecondSpike.setLinearHeadingInterpolation(shootPose.getHeading(), secondSpikeInitial.getHeading());

        secondSpike = new Path(new BezierLine(secondSpikeInitial, secondSpikeFinal));
        secondSpike.setLinearHeadingInterpolation(secondSpikeInitial.getHeading(), secondSpikeFinal.getHeading());

        shootFromSecondSpike = new Path(new BezierLine(secondSpikeFinal, shootPose));
        shootFromSecondSpike.setLinearHeadingInterpolation(secondSpikeFinal.getHeading(), shootPose.getHeading());

        shootToThirdSpike = new Path(new BezierLine(shootPose, thirdSpikeInitial));
        shootToThirdSpike.setLinearHeadingInterpolation(shootPose.getHeading(), thirdSpikeInitial.getHeading());

        thirdSpike = new Path(new BezierLine(thirdSpikeInitial, thirdSpikeFinal));
        thirdSpike.setLinearHeadingInterpolation(thirdSpikeInitial.getHeading(), thirdSpikeFinal.getHeading());

        shootFromThirdSpike = new Path(new BezierLine(thirdSpikeFinal, shootPose));
        shootFromThirdSpike.setLinearHeadingInterpolation(thirdSpikeFinal.getHeading(), shootPose.getHeading());

        goToEnd = new Path(new BezierLine(shootPose, endPose));
        goToEnd.setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading());
    }

    private void buildSpikeArrays() {
        moveToSpike = new Path[]{
                shootToFirstSpike,
                shootToSecondSpike,
                shootToThirdSpike
        };

        intakeSpike = new Path[]{
                firstSpike,
                secondSpike,
                thirdSpike
        };

        returnFromSpike = new Path[]{
                shootFromFirstSpike,
                shootFromSecondSpike,
                shootFromThirdSpike
        };
    }

    @Override
    public void start() {
        limelight.start();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case PRE_MOVE:
                if (!follower.isBusy()) {
                    follower.followPath(preMove);
                    pathTimer.resetTimer();
                    state = AutoState.LIMELIGHT_ALIGN;
                }
                break;

            case LIMELIGHT_ALIGN:
                handleLimelightAlign();
                break;

            case SHOOT_INITIAL:
                if (!follower.isBusy()) {
                    //Shoot(3, 3);
                    state = AutoState.MOVE_TO_SPIKE;
                }
                break;

            case MOVE_TO_SPIKE:
                followAndAdvance(moveToSpike[currentSpike], AutoState.INTAKE_SPIKE);
                break;

            case INTAKE_SPIKE:
                intakeAndAdvance(intakeSpike[currentSpike]);
                break;

            case RETURN_FROM_SPIKE:
                returnAndAdvance(returnFromSpike[currentSpike]);
                break;

            case SHOOT_SPIKE:
                if (!follower.isBusy()) {
                    //Shoot(shootStateIds[currentSpike], 3);
                    currentSpike++;

                    if (currentSpike >= moveToSpike.length) {
                        state = AutoState.GO_TO_END;
                    } else {
                        state = AutoState.MOVE_TO_SPIKE;
                    }
                }
                break;

            case GO_TO_END:
                followAndAdvance(goToEnd, AutoState.DONE);
                break;

            case DONE:
                break;
        }
    }

    private void handleLimelightAlign() {
        if (limelight.isRunning()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                limelightPose = result.getBotpose();
            }
        }

        if (limelightPose != null) {
            double x = -10 - (limelightPose.getPosition().y * 39.37);
            double y = 159.7 + (limelightPose.getPosition().x * 39.37);
            double heading = Math.toRadians(limelightPose.getOrientation().getYaw() - 90);

            Pose startPose = new Pose(x, y, heading);
            Path moveToShoot = new Path(new BezierLine(startPose, shootPose));
            moveToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

            follower.followPath(moveToShoot);
            limelight.stop();
            servoCamera.setPosition(1.0);
            state = AutoState.SHOOT_INITIAL;

        } else if (pathTimer.getElapsedTimeSeconds() > 2.0) {
            Pose fallbackPose = follower.getPose();
            Path moveToShoot = new Path(new BezierLine(fallbackPose, shootPose));
            moveToShoot.setLinearHeadingInterpolation(fallbackPose.getHeading(), shootPose.getHeading());

            follower.followPath(moveToShoot);
            limelight.stop();
            pathTimer.resetTimer();
            state = AutoState.SHOOT_INITIAL;
        }
    }

    private void followAndAdvance(Path path, AutoState next) {
        if (!follower.isBusy()) {
            follower.followPath(path);
            state = next;
        }
    }

    private void intakeAndAdvance(Path path) {
        if (!follower.isBusy()) {
            intake.setPower(1);
            follower.followPath(path);
            state = AutoState.RETURN_FROM_SPIKE;
        }
    }

    private void returnAndAdvance(Path path) {
        if (!follower.isBusy()) {
            intake.setPower(0);
            follower.followPath(path);
            pathTimer.resetTimer();
            state = AutoState.SHOOT_SPIKE;
        }
    }
}
*/