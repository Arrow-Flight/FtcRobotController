package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.getCorrectedPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.getPoseFromLimelight;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.vel;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@TeleOp
public class MainOpBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Set Up Camera Servo
        Servo servoCamera = hardwareMap.get(Servo.class, "servoCamera");
        servoCamera.scaleRange(0.3, 1.0);
        servoCamera.setPosition(0.4);

        //Set Up Intake
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set Up Upper
        DcMotor upper = hardwareMap.get(DcMotor.class, "upper");
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Up Shooter Motors
        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(42.0, 0, 0, 18.5);
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Up Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);

        // Set Up Follower
        Pose start = new Pose(35, 75, Math.toRadians(0));
        Pose shootAt = new Pose(52, 96, Math.toRadians(-50));
        Pose currentPose;
        Path toShoot;
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        follower.update();

        // Variables
        boolean shooting = false;
        boolean xSpin = false;
        int pathState = 0;
        int shooterTargetVelocity = 1200;
        int currentState = 0;
        Pose exPose = new Pose(130, 65, Math.toRadians(0));
        int exR = 30;
        double diffX;
        double diffY;
        boolean inZone = false;
        double unitX;
        double unitY;
        double escX;
        double escY;
        double distance;
        boolean insideZone;
        boolean escapingZone = false;
        Pose escPose;
        Path escPath;
        Timer pathTimer;
        pathTimer = new Timer();


        waitForStart();
        while (opModeIsActive()) {
            currentPose = follower.getPose();
            follower.update();

            diffX = currentPose.getX() - exPose.getX();
            diffY = currentPose.getY() - exPose.getY();
            distance = Math.sqrt(diffX * diffX + diffY * diffY);

            insideZone = distance < exR;

            if (insideZone && !escapingZone) {
                escapingZone = true;

                unitX = diffX / distance;
                unitY = diffY / distance;

                escX = currentPose.getX() + unitX * 10;
                escY = currentPose.getY() + unitY * 10;
                escPose = new Pose(escX, escY, currentPose.getHeading());

                escPath = new Path(new BezierLine(currentPose, escPose));
                escPath.setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        currentPose.getHeading()
                );

                follower.followPath(escPath);
            }

            if (escapingZone && !follower.isBusy()) {
                follower.breakFollowing();
                escapingZone = false;
            }



            telemetry.addData("Pose", follower.getPose());
                telemetry.addData("xSpin", xSpin);
                telemetry.addData("inZone", inZone);
                telemetry.addData("diffX", diffX);
                telemetry.addData("diffY", diffY);
                telemetry.addData("Distance", distance);
                telemetry.update();
                vel = (shooterRight.getVelocity() + shooterLeft.getVelocity()) / 2;


                if (!escapingZone) {

                    double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                    double x = gamepad1.left_stick_x;
                    double rx = gamepad1.right_stick_x;


                    double botHeading = follower.getHeading();


                    double rotX = x * Math.cos(-botHeading) + y * Math.sin(botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(botHeading);

                    rotX = rotX * 1.1;


                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    frontRight.setPower(frontRightPower);
                    backRight.setPower(backRightPower);

                    if (gamepad1.yWasPressed()) {
                        if (shooting) {
                            shooting = false;
                            follower.breakFollowing();
                        } else {
                            shooting = true;
                            pathState = 0;
                        }
                    }


                    if (!shooting) {
                        if (gamepad1.xWasPressed()) {
                            xSpin = !xSpin;
                        }

                        if (xSpin) {
                            shooterLeft.setVelocity(shooterTargetVelocity);
                            shooterRight.setVelocity(shooterTargetVelocity);
                        } else {
                            shooterLeft.setVelocity(0);
                            shooterRight.setVelocity(0);
                        }

                        if (gamepad1.left_bumper) {
                            intake.setPower(-1);
                        } else {
                            intake.setPower(gamepad1.left_trigger);
                        }

                        if (gamepad1.right_bumper) {
                            upper.setPower(-1);
                        } else {
                            upper.setPower(gamepad1.right_trigger);
                        }
                    }


                    if (shooting) {
                        if (pathState == 0 && !follower.isBusy()) {

                            toShoot = new Path(new BezierLine(currentPose, shootAt));
                            toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), shootAt.getHeading());

                            follower.followPath(toShoot);
                            pathState = 1;
                            pathTimer.resetTimer();

                        } else if (pathState == 1 && !follower.isBusy()) {
                            if (pathTimer.getElapsedTimeSeconds() > 2) {

                                Pose llPose = getPoseFromLimelight();

                                if (llPose != null) {
                                    telemetry.addData("Result:", llPose);

                                    Pose corrected = getCorrectedPose(llPose, shootAt);
                                    toShoot = new Path(new BezierLine(corrected, shootAt));
                                    toShoot.setLinearHeadingInterpolation(corrected.getHeading(), shootAt.getHeading());

                                    follower.followPath(toShoot);

                                } else {
                                    telemetry.addData("Result:", null);
                                }
                                telemetry.update();

                                pathState = 2;
                            }
                        } else if (pathState == 2 && !follower.isBusy()) {
                            shooterLeft.setVelocity(shooterTargetVelocity);
                            shooterRight.setVelocity(shooterTargetVelocity);

                            if (vel >= 1200 && currentState == 0) {
                                currentState = 1;
                            } else if (currentState == 1) {
                                upper.setPower(1);
                                intake.setPower(1);

                                if (vel <= 1100) {
                                    upper.setPower(0);
                                    intake.setPower(0);
                                    currentState = 0;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
