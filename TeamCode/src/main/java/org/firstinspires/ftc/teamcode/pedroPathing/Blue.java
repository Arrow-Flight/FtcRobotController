package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Blue.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.ShooterPIDF.*;

import com.pedropathing.geometry.*;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@Autonomous
public class Blue extends OpMode {

    private enum AutoState {
        PRE_MOVE,
        LIMELIGHT_ALIGN,
        SHOOT_INITIAL,

        MOVE_FIRST_SPIKE,
        INTAKE_FIRST_SPIKE,
        RETURN_FIRST_SPIKE,
        SHOOT_FIRST,

        MOVE_SECOND_SPIKE,
        INTAKE_SECOND_SPIKE,
        RETURN_SECOND_SPIKE,
        SHOOT_SECOND,

        MOVE_THIRD_SPIKE,
        INTAKE_THIRD_SPIKE,
        RETURN_THIRD_SPIKE,
        SHOOT_THIRD,

        GO_TO_END,
        DONE
    }

    private AutoState state = AutoState.PRE_MOVE;

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

    @Override
    public void start() {
        limelight.start();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        updateShooterPIDF();
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
                shootAndAdvance(AutoState.MOVE_FIRST_SPIKE, 3, 3);
                break;

            case MOVE_FIRST_SPIKE:
                followAndAdvance(shootToFirstSpike, AutoState.INTAKE_FIRST_SPIKE);
                break;

            case INTAKE_FIRST_SPIKE:
                intakeAndAdvance(firstSpike, AutoState.RETURN_FIRST_SPIKE);
                break;

            case RETURN_FIRST_SPIKE:
                returnAndAdvance(shootFromFirstSpike, AutoState.SHOOT_FIRST);
                break;

            case SHOOT_FIRST:
                shootAndAdvance(AutoState.MOVE_SECOND_SPIKE, 7, 3);
                break;

            case MOVE_SECOND_SPIKE:
                followAndAdvance(shootToSecondSpike, AutoState.INTAKE_SECOND_SPIKE);
                break;

            case INTAKE_SECOND_SPIKE:
                intakeAndAdvance(secondSpike, AutoState.RETURN_SECOND_SPIKE);
                break;

            case RETURN_SECOND_SPIKE:
                returnAndAdvance(shootFromSecondSpike, AutoState.SHOOT_SECOND);
                break;

            case SHOOT_SECOND:
                shootAndAdvance(AutoState.MOVE_THIRD_SPIKE, 11, 3);
                break;

            case MOVE_THIRD_SPIKE:
                followAndAdvance(shootToThirdSpike, AutoState.INTAKE_THIRD_SPIKE);
                break;

            case INTAKE_THIRD_SPIKE:
                intakeAndAdvance(thirdSpike, AutoState.RETURN_THIRD_SPIKE);
                break;

            case RETURN_THIRD_SPIKE:
                returnAndAdvance(shootFromThirdSpike, AutoState.SHOOT_THIRD);
                break;

            case SHOOT_THIRD:
                shootAndAdvance(AutoState.GO_TO_END, 15, 3);
                break;

            case GO_TO_END:
                followAndAdvance(goToEnd, AutoState.DONE);
                break;

            case DONE:
                break;
        }
    }

    private void updateShooterPIDF() {
        double currentError = shooterTargetVelocity - shooterRight.getVelocity();
        shooterTargetPower = (shooterF * shooterTargetVelocity)
                + (shooterP * currentError)
                + (shooterD * (currentError - previousError));
        previousError = currentError;
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

    private void intakeAndAdvance(Path path, AutoState next) {
        if (!follower.isBusy()) {
            intake.setPower(1);
            follower.followPath(path);
            state = next;
        }
    }

    private void returnAndAdvance(Path path, AutoState next) {
        if (!follower.isBusy()) {
            intake.setPower(0);
            follower.followPath(path);
            pathTimer.resetTimer();
            state = next;
        }
    }

    private void shootAndAdvance(AutoState next, int stateId, int count) {
        if (!follower.isBusy()) {
            Shoot(stateId, count);
            state = next;
        }
    }
}
