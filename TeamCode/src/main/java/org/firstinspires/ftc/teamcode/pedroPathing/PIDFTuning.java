package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.PipelineRecordingParameters.*;

@TeleOp
@Config
public class PIDFTuning extends OpMode {

    public DcMotor left;
    public DcMotor right;
    public DcMotorEx throughBore;
    PIDFController pidfController;

    double highVelocity = 2800;
    double lowVelocity = 2000;

    double curTargetVelocity = highVelocity;
    double P = 0;
    double F = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};

    int stepIndex = 1;
    public static double curVelocity;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "shooterLeft");
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        right = hardwareMap.get(DcMotor.class, "shooterRight");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        throughBore = hardwareMap.get(DcMotorEx.class, "intake");

        pidfController = new PIDFController(new com.pedropathing.control.PIDFCoefficients(P, 0, 0, F));
        pidfController.setTargetPosition(curTargetVelocity);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasReleased()) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
            pidfController.setTargetPosition(curTargetVelocity);
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

        curVelocity = getVelocity();

        pidfController.setP(P);
        pidfController.setF(F);
        double normalizedTarget = curTargetVelocity;
        pidfController.setTargetPosition(normalizedTarget);
        pidfController.updatePosition(curVelocity);
        pidfController.updateFeedForwardInput(1.0);

        double prePower = pidfController.run();

        double power = Math.max(-1.0, Math.min(1.0, prePower));

        left.setPower(power);
        right.setPower(power);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("Target velocity", curTargetVelocity);
        dashboardTelemetry.addData("Current velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", pidfController.getError());
        telemetry.addLine("-------------------------");
        telemetry.addData("Tuning P", "%.6f (D-Pad U/D", P);
        telemetry.addData("Tuning F", "%.6f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.6f (B Button", stepSizes[stepIndex]);
        dashboardTelemetry.update();
        telemetry.update();
    }

    private long lastPosition = 0;
    private long lastTime = 0;
    private static final int VELOCITY_BUFFER_SIZE = 5;
    private final double[] velocityBuffer = new double[VELOCITY_BUFFER_SIZE];
    private int bufferIndex = 0;
    private int bufferFilled = 0;

    double getVelocity() {
        long currentPosition = throughBore.getCurrentPosition();
        long currentTime = System.nanoTime();

        if (lastTime == 0) {
            lastPosition = currentPosition;
            lastTime = currentTime;
            return 0;
        }

        long delta = currentPosition - lastPosition;
        double deltaTime = (currentTime - lastTime) / 1e9;

        // Guard against tiny deltaTime causing velocity spikes
        if (deltaTime < 0.001) {
            lastTime = currentTime;
            double weightedSum = 0, weightSum = 0;
            for (int i = 0; i < bufferFilled; i++) {
                int age = (bufferIndex - 1 - i + VELOCITY_BUFFER_SIZE) % VELOCITY_BUFFER_SIZE;
                double weight = bufferFilled - age;
                weightedSum += velocityBuffer[i] * weight;
                weightSum += weight;
            }
            return weightSum > 0 ? weightedSum / weightSum : 0;
        }

        double instantVelocity = (delta / deltaTime / 8192.0) * 60.0;

        // Spike rejection
        double currentAverage = 0;
        if (bufferFilled > 0) {
            double sum = 0;
            for (int i = 0; i < bufferFilled; i++) sum += velocityBuffer[i];
            currentAverage = sum / bufferFilled;
        }

        if (bufferFilled == 0 || Math.abs(instantVelocity - currentAverage) < 100) {
            velocityBuffer[bufferIndex] = instantVelocity;
            bufferIndex = (bufferIndex + 1) % VELOCITY_BUFFER_SIZE;
            if (bufferFilled < VELOCITY_BUFFER_SIZE) bufferFilled++;
        }

        // Weighted average
        double weightedSum = 0, weightSum = 0;
        for (int i = 0; i < bufferFilled; i++) {
            int age = (bufferIndex - 1 - i + VELOCITY_BUFFER_SIZE) % VELOCITY_BUFFER_SIZE;
            double weight = bufferFilled - age;
            weightedSum += velocityBuffer[i] * weight;
            weightSum += weight;
        }

        lastPosition = currentPosition;
        lastTime = currentTime;

        return weightSum > 0 ? weightedSum / weightSum : 0;
    }
}