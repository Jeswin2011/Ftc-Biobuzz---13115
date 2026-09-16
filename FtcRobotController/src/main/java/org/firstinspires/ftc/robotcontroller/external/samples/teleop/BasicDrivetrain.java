package org.firstinspires.ftc.robotcontroller.external.samples.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Nothing calls this class by name. The Driver Station finds it at run time by
// scanning for the @TeleOp annotation, so the IDE reporting it as unused is wrong.
@SuppressWarnings("unused")
@TeleOp(name = "Basic Drivetrain", group = "Drive")
public class BasicDrivetrain extends LinearOpMode {

    // goBILDA 5203 Yellow Jacket, 19.2:1 gearbox, 312 RPM at 12V. The encoder sits
    // on the motor shaft ahead of the gearbox and gives 28 counts per motor turn,
    // so one turn of the output shaft is 28 * 19.2031 = 537.7 counts.
    private static final double TICKS_PER_REV = 537.7;

    // True means the hub holds each wheel at the speed the driver asked for, which
    // needs the encoder cables plugged in - if they are not, the robot will not
    // move at all. False drives the motors directly and needs no encoder cables,
    // at the cost of the hub no longer correcting motor-to-motor variation.
    private static final boolean USE_ENCODERS = true;

    // Sticks rest a little off centre, so ignore anything this small. The SDK
    // passes stick values through untouched, so this is the only deadband.
    private static final double DEADBAND = 0.05;

    // Mecanum wheels strafe slower than they drive, and full-stick turns are
    // twitchy. These even out how the robot responds in each direction.
    private static final double STRAFE_GAIN = 1.1;
    private static final double TURN_GAIN = 0.8;

    // Biggest power change allowed per second. A full-stick reversal is a 2.0 jump,
    // and four of these motors reversing at once sags the battery enough to drop
    // the connection. At 8.0 a stop-to-full push takes an eighth of a second and a
    // full reversal takes a quarter, neither of which the driver can feel.
    private static final double SLEW_PER_SECOND = 8.0;

    // Telemetry is the slowest thing in the loop, so rebuild it 10x a second
    // instead of every cycle.
    private static final double TELEMETRY_INTERVAL = 0.1;

    // BRAKE stops the robot where the driver lets go. Swap to FLOAT to coast.
    private static final DcMotor.ZeroPowerBehavior STOPPING = DcMotor.ZeroPowerBehavior.BRAKE;

    @Override
    public void runOpMode() {
        // Control hub motor ports: front_left_drive = 2, front_right_drive = 1,
        // back_left_drive = 3, back_right_drive = 0.
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        // Fetches every encoder in one hub transaction instead of one round trip
        // per read, which keeps the telemetry below from slowing the loop down.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // The power maths below assumes positive power drives every wheel forward.
        // The two sides face opposite ways, so one of them has to be reversed. If
        // the robot drives backwards, swap REVERSE and FORWARD here.
        configure(frontLeft, DcMotor.Direction.REVERSE);
        configure(backLeft, DcMotor.Direction.REVERSE);
        configure(frontRight, DcMotor.Direction.FORWARD);
        configure(backRight, DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", USE_ENCODERS ? "closed loop" : "open loop");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime telemetryTimer = new ElapsedTime();

        double lastFrontLeft = 0.0;
        double lastFrontRight = 0.0;
        double lastBackLeft = 0.0;
        double lastBackRight = 0.0;

        while (opModeIsActive()) {
            // Rate limiting has to be based on real elapsed time, not loop count,
            // or the robot feels different whenever the loop speed changes.
            double maxDelta = SLEW_PER_SECOND * loopTimer.seconds();
            loopTimer.reset();

            double drive = shape(-gamepad1.left_stick_y);
            double strafe = shape(gamepad1.left_stick_x) * STRAFE_GAIN;
            double turn = shape(gamepad1.right_stick_x) * TURN_GAIN;

            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            // Only scale down when a wheel would be over 1.0. Dividing by the raw
            // max instead would make small stick movements jump to full speed.
            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

            lastFrontLeft = slew(frontLeftPower / max, lastFrontLeft, maxDelta);
            lastFrontRight = slew(frontRightPower / max, lastFrontRight, maxDelta);
            lastBackLeft = slew(backLeftPower / max, lastBackLeft, maxDelta);
            lastBackRight = slew(backRightPower / max, lastBackRight, maxDelta);

            frontLeft.setPower(lastFrontLeft);
            frontRight.setPower(lastFrontRight);
            backLeft.setPower(lastBackLeft);
            backRight.setPower(lastBackRight);

            if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL) {
                telemetryTimer.reset();
                telemetry.addData("Power front", "L %.2f  R %.2f", lastFrontLeft, lastFrontRight);
                telemetry.addData("Power back", "L %.2f  R %.2f", lastBackLeft, lastBackRight);
                // A wheel reading 0 RPM while its power is not 0 means that motor's
                // encoder cable is loose.
                telemetry.addData("RPM front", "L %.0f  R %.0f", rpm(frontLeft), rpm(frontRight));
                telemetry.addData("RPM back", "L %.0f  R %.0f", rpm(backLeft), rpm(backRight));
                telemetry.update();
            }
        }
    }

    private static void configure(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(STOPPING);
        motor.setMode(USE_ENCODERS
                ? DcMotor.RunMode.RUN_USING_ENCODER
                : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Output-shaft speed in RPM. */
    private static double rpm(DcMotorEx motor) {
        return motor.getVelocity() / TICKS_PER_REV * 60.0;
    }

    /** Removes stick drift and softens the centre of the stick without losing top speed. */
    private static double shape(double stickValue) {
        if (Math.abs(stickValue) < DEADBAND) {
            return 0.0;
        }
        // Rescale so the stick still reaches a full 1.0 once the deadband is gone,
        // otherwise the robot would never quite reach top speed.
        double scaled = (Math.abs(stickValue) - DEADBAND) / (1.0 - DEADBAND);
        return Math.copySign(scaled * scaled, stickValue);
    }

    /** Moves previous towards target by at most maxDelta. */
    private static double slew(double target, double previous, double maxDelta) {
        return previous + Range.clip(target - previous, -maxDelta, maxDelta);
    }
}
