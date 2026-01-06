package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp
public class MainOp extends LinearOpMode {

    private DcMotor shooterRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor intake;
    private DcMotor shooterLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor break_DcMotor;

    double power_divisor;
    List active;
    boolean on;
    double target_power;
    double previous_error;
    boolean slow;

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        break_DcMotor = hardwareMap.get(DcMotor.class, "break");

        waitForStart();
        Initialization();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Get_Controller_Inputs();
                Shooter_PIDF();
                Actuators();
                Call_Telemetry();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Shooter_PIDF() {
        double p;
        double f;
        double d;
        double current_error;
        int target_velocity = 0;

        p = 0.001;
        f = 0.00045;
        d = 0.005;
        current_error = target_velocity - ((DcMotorEx) shooterRight).getVelocity();
        target_velocity = 1200;
        target_power = f * target_velocity + p * (target_velocity - ((DcMotorEx) shooterRight).getVelocity()) + d * (current_error - previous_error);
        previous_error = current_error;
    }

    /**
     * Describe this function...
     */
    private void Initialization() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        power_divisor = 1.5;
        on = false;
        slow = false;
    }

    /**
     * Describe this function...
     */
    private void Get_Controller_Inputs() {
        double leftStick;

        leftStick = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180;
        if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
            // Center
            active = JavaUtil.createListWith(0, 0, 0, 0);
        } else if (leftStick >= 157.5 || leftStick <= -157.5) {
            // Left
            active = JavaUtil.createListWith(-1, 1, 1, -1);
        } else if (leftStick > 112.5) {
            // Up Left
            active = JavaUtil.createListWith(0, 1, 1, 0);
        } else if (leftStick > 67.5) {
            // Up
            active = JavaUtil.createListWith(1, 1, 1, 1);
        } else if (leftStick > 22.5) {
            // Up Right
            active = JavaUtil.createListWith(1, 0, 0, 1);
        } else if (leftStick > -22.5) {
            // Right
            active = JavaUtil.createListWith(1, -1, -1, 1);
        } else if (leftStick > -67.5) {
            // Down Right
            active = JavaUtil.createListWith(0, -1, -1, 0);
        } else if (leftStick > -112.5) {
            // Down
            active = JavaUtil.createListWith(-1, -1, -1, -1);
        } else if (leftStick > -157.5) {
            // Down Left
            active = JavaUtil.createListWith(-1, 0, 0, -1);
        } else {
            // Center
            active = JavaUtil.createListWith(0, 0, 0, 0);
        }
    }

    /**
     * Describe this function...
     */
    private void Call_Telemetry() {
        telemetry.addData("Left", ((DcMotorEx) shooterLeft).getVelocity());
        telemetry.addData("Right", ((DcMotorEx) shooterRight).getVelocity());
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void Actuators() {
        frontLeft.setPower((((Double) JavaUtil.inListGet(active, JavaUtil.AtMode.FROM_START, (int) 0, false)).doubleValue() + (gamepad1.right_stick_x - gamepad1.right_stick_y)) / power_divisor);
        frontRight.setPower((((Double) JavaUtil.inListGet(active, JavaUtil.AtMode.FROM_START, (int) 1, false)).doubleValue() + (-gamepad1.right_stick_x - gamepad1.right_stick_y)) / power_divisor);
        backLeft.setPower((((Double) JavaUtil.inListGet(active, JavaUtil.AtMode.FROM_START, (int) 2, false)).doubleValue() + (gamepad1.right_stick_x - gamepad1.right_stick_y)) / power_divisor);
        backRight.setPower((((Double) JavaUtil.inListGet(active, JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue() + (-gamepad1.right_stick_x - gamepad1.right_stick_y)) / power_divisor);
        if (on) {
            shooterRight.setPower(target_power);
            shooterLeft.setPower(target_power);
        } else {
            shooterRight.setPower(0);
            shooterLeft.setPower(0);
        }
        if (gamepad1.yWasReleased()) {
            on = !on;
            previous_error = 0;
        }
        if (gamepad1.left_bumper) {
            intake.setPower(-0.5);
        } else {
            intake.setPower(gamepad1.left_trigger);
        }
        if (gamepad1.right_bumper) {
            break_DcMotor.setPower(-0.5);
        } else {
            break_DcMotor.setPower(gamepad1.right_trigger);
        }
        if (slow) {
            power_divisor = 6;
        } else {
            power_divisor = 1;
        }
        if (gamepad1.xWasReleased()) {
            slow = !slow;
        }
    }
}