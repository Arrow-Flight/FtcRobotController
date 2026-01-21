package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "MainOp (Blocks to Java)")
public class MainOp extends LinearOpMode {

  private DcMotor frontLeft;
  private DcMotor backLeft;
  private DcMotor shooterRight;
  private DcMotor intake;
  private DcMotor shooterLeft;
  private DcMotor backRight;
  private DcMotor frontRight;
  private DcMotor upper;

  double power_divisor;
  boolean on;
  PIDFCoefficients pidfCoefficients;
  boolean slow;
  int target_velocity;
  List active;

  /**
   * Describe this function...
   */
  @Override
  public void runOpMode() {
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
    intake = hardwareMap.get(DcMotor.class, "intake");
    shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    upper = hardwareMap.get(DcMotor.class, "upper");

    waitForStart();
    Initialization();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        Get_Controller_Inputs();
        Actuators();
        Call_Telemetry();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Initialization() {
    power_divisor = 1.5;
    on = false;
    slow = false;
    target_velocity = 1200;
    pidfCoefficients = new PIDFCoefficients(42, 0, 0, 18.5, MotorControlAlgorithm.PIDF);
    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    backLeft.setDirection(DcMotor.Direction.REVERSE);
    shooterRight.setDirection(DcMotor.Direction.REVERSE);
    intake.setDirection(DcMotor.Direction.REVERSE);
    shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ((DcMotorEx) shooterLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    ((DcMotorEx) shooterRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
    ((DcMotorEx) shooterLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    ((DcMotorEx) shooterRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    frontLeft.setPower((((Double) JavaUtil.inListGet(active, JavaUtil.AtMode.FROM_START, (int) 0, false)).doubleValue() + (gamepad1.right_stick_x - gamepad1.right_stick_y)) / power_divisor);
    frontRight.setPower((((Double) JavaUtil.inListGet(active, JavaUtil.AtMode.FROM_START, (int) 1, false)).doubleValue() + (-gamepad1.right_stick_x - gamepad1.right_stick_y)) / power_divisor);
    backLeft.setPower((((Double) JavaUtil.inListGet(active, JavaUtil.AtMode.FROM_START, (int) 2, false)).doubleValue() + (gamepad1.right_stick_x - gamepad1.right_stick_y)) / power_divisor);
    backRight.setPower((((Double) JavaUtil.inListGet(active, JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue() + (-gamepad1.right_stick_x - gamepad1.right_stick_y)) / power_divisor);
    if (on) {
      ((DcMotorEx) shooterRight).setVelocity(target_velocity);
      ((DcMotorEx) shooterLeft).setVelocity(target_velocity);
    } else {
      shooterRight.setPower(0);
      shooterLeft.setPower(0);
    }
    if (gamepad1.y_was_released()) {
      on = !on;
    }
    if (gamepad1.left_bumper) {
      intake.setPower(-0.5);
    } else {
      intake.setPower(gamepad1.left_trigger);
    }
    if (gamepad1.right_bumper) {
      upper.setPower(-0.5);
    } else {
      upper.setPower(gamepad1.right_trigger);
    }
    if (slow) {
      power_divisor = 6;
    } else {
      power_divisor = 1;
    }
    if (gamepad1.x_was_released()) {
      slow = !slow;
    }
    telemetry.addData("Velocity", (((DcMotorEx) shooterLeft).getVelocity() + ((DcMotorEx) shooterRight).getVelocity()) / 2);
    telemetry.update();
  }
}
