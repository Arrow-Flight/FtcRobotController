package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.getVelocity;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.pidfController;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@TeleOp
public class MainOpRed extends LinearOpMode {
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
        DcMotor shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        DcMotor shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
        Pose start = new Pose(105,75, Math.toRadians(180));
        Pose currentPose;
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(start);
        follower.update();

        // Variables
        boolean shooting = false;
        boolean xSpin = false;
        Pose exPose = new Pose(8, 84, Math.toRadians(180));
        int exR = 24;
        int slowing = 1;
        double diffX;
        double diffY;
        double unitX;
        double unitY;
        double escX;
        double escY;
        double distance;
        boolean insideZone;
        boolean escapingZone = false;
        boolean shootingPath = false;
        boolean boxing = false;
        Pose escPose;
        Path escPath;
        pidfController = new PIDFController(new com.pedropathing.control.PIDFCoefficients(0.002, 0, 0, 0.72));
        pidfController.setTargetPosition(2800);


        waitForStart();
        while (opModeIsActive()) {
            currentPose = follower.getPose();
            follower.update();
            double vel = getVelocity();
            telemetry.addData("vel", vel);
            telemetry.update();

            pidfController.updatePosition(vel);
            pidfController.updateFeedForwardInput(pidfController.F());

            double prePower = pidfController.run();
            double power = Math.max(-1.0, Math.min(1.0, prePower));

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


            if (!escapingZone) {

                    double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                    double x = gamepad1.left_stick_x;
                    double rx = gamepad1.right_stick_x;


                    double botHeading = follower.getHeading();


                    double rotX = x * Math.cos(-botHeading) + y * Math.sin(botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(botHeading);

                    rotX = rotX * 1.1;


                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1)  * slowing;
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    frontRight.setPower(frontRightPower);
                    backRight.setPower(backRightPower);

                if (gamepad1.yWasPressed()) {
                    follower.breakFollowing();
                    shooting = !shooting;
                }

                if (gamepad1.bWasPressed()) {
                    follower.breakFollowing();
                    boxing = !boxing;
                }
                if (boxing) {
                    slowing = 6;
                } else slowing = 1;

                if (shooting) {
                    boolean driverControlling = Math.abs(gamepad1.left_stick_x) > 0.05
                            || Math.abs(gamepad1.left_stick_y) > 0.05
                            || Math.abs(gamepad1.right_stick_x) > 0.05;

                    if (driverControlling) {
                        follower.breakFollowing();
                        shootingPath = false; // let driver take over
                    } else if (!shootingPath || !follower.isBusy()) {
                        Pose goTo = getShootPose(currentPose);
                        double dx = currentPose.getX() - goTo.getX();
                        double dy = currentPose.getY() - goTo.getY();
                        if (Math.sqrt(dx*dx + dy*dy) > 3) {
                            Path shootPath = new Path(new BezierLine(currentPose, goTo));
                            shootPath.setLinearHeadingInterpolation(currentPose.getHeading(), goTo.getHeading());
                            follower.followPath(shootPath);
                            shootingPath = true;
                        }
                    }
                }

                if (gamepad1.yWasPressed()) {
                    follower.breakFollowing();
                    shooting = !shooting;
                    shootingPath = false;
                }

                if (gamepad1.xWasPressed()) {
                    xSpin = !xSpin;
                }
                if (xSpin) {
                    shooterLeft.setPower(power);
                    shooterRight.setPower(power);
                } else {
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                }

                if (gamepad1.left_bumper) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(gamepad1.left_trigger);
                }

                if (gamepad1.right_bumper) {
                    upper.setPower(-1);
                } else {
                    upper.setPower(Math.min(gamepad1.right_trigger, 0.75));
                }
            }
        }
    }
    public static Pose getShootPose(Pose current) {
        double goalX = 132;
        double goalY = 136;
        double radius = 40;
        double firstAngle = -135;
        double secondAngle = 170;

        double dx = current.getX() - goalX;
        double dy = current.getY() - goalY; // will be negative since robot is below goal

        double angle = Math.atan2(dy, dx);
        double deg = Math.toDegrees(angle);

        double clampDeg;

        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;

        boolean inArc = deg >= -180 && deg <= -firstAngle || deg >= secondAngle && deg <= 180;

        if (inArc) {
            clampDeg = deg;
        } else {
            double distToFirstAngle = Math.abs(deg - (firstAngle));
            if (distToFirstAngle > 180) distToFirstAngle = 360 - distToFirstAngle;
            double distToSecondAngle = Math.abs(deg - secondAngle);
            if (distToSecondAngle > 180) distToSecondAngle = 360 - distToSecondAngle;

            clampDeg = (distToFirstAngle < distToSecondAngle) ? firstAngle : secondAngle;
        }

        double angleRad = Math.toRadians(clampDeg);

        double x = goalX + radius * Math.cos(angleRad);
        double y = goalY + radius * Math.sin(angleRad);

        double headingToGoal = Math.atan2(-Math.cos(angleRad), -Math.sin(angleRad));
        return new Pose(x, y, (-headingToGoal - (Math.PI/2)));
    }
}