package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDFTuning extends OpMode {

    public DcMotorEx left;
    public DcMotorEx right;

    double highVelocity = 1500;
    double lowVelocity = 900;

    double curTargetVelocity = highVelocity;
    double P = 0;
    double F = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        right = hardwareMap.get(DcMotorEx.class, "shooterRight");
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(42.0, 0, 0, 13.5329);
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasReleased()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        left.setVelocity(curTargetVelocity);
        right.setVelocity(curTargetVelocity);

        double curVelocity = (right.getVelocity() + left.getVelocity())/2;
        double error = curTargetVelocity - curVelocity;


        telemetry.addData("Target velocity", curTargetVelocity);
        telemetry.addData("Current velocity","%.2f", curVelocity);
        telemetry.addData("Error","%.2f", error);
        telemetry.addLine("-------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D", P);
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size","%.4f (B Button", stepSizes[stepIndex]);
    }
}
