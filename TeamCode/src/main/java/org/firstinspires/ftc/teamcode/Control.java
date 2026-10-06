package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class Control extends LinearOpMode {
    private DcMotor left_front;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor left_back;

    @Override
    public void runOpMode() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "rightFront");
        right_back = hardwareMap.get(DcMotor.class, "rightBack");

        // Reverse one side (swap to the left side if your robot goes backward)
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;   // forward/back
            double x = gamepad1.left_stick_x;    // strafe
            double rx = gamepad1.right_stick_x;  // turn

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            left_front.setPower((y + x + rx) / denominator);
            left_back.setPower((y - x + rx) / denominator);
            right_front.setPower((y - x - rx) / denominator);
            right_back.setPower((y + x - rx) / denominator);
        }
    }
}