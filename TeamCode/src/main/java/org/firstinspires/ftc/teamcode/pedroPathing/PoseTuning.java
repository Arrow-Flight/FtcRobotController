package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Blue.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.ShooterPIDF.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@Autonomous
public class PoseTuning extends OpMode {

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
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
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
        double currentError = (shooterTargetVelocity - shooterRight.getVelocity());
        shooterTargetPower = ((shooterF * shooterTargetVelocity) + (shooterP * (shooterTargetVelocity - shooterRight.getVelocity())) + (shooterD * (currentError - previousError)));
        follower.update();
        FtcDashboard.getInstance().getTelemetry().addData("Left", shooterLeft.getVelocity());
        FtcDashboard.getInstance().getTelemetry().addData("Right", shooterRight.getVelocity());
        FtcDashboard.getInstance().getTelemetry().update();

        if (gamepad1.right_bumper) {
            shooterLeft.setPower(shooterTargetPower);
            shooterRight.setPower(shooterTargetPower);
        } else if (gamepad1.left_bumper) {
            shooterLeft.setVelocity(1200);
            shooterRight.setVelocity(1200);
        } else {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }

        previousError = currentError;
    }
}