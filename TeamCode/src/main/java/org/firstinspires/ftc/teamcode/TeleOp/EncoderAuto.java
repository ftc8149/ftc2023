package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Encoder Auto", group="Robot")
@Disabled
public class EncoderAuto extends LinearOpMode {

    private DcMotor FL = null;
    private DcMotor FR = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7; //GoBilda Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0; //No Gear Reduction
    static final double WHEEL_DIAMETER_INCHES = 3.77953; //GoBilda Mecanum Wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at", "%7d :%7d",
                FL.getCurrentPosition(),
                FR.getCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED, 48, 48, 5.0);
        encoderDrive(TURN_SPEED, 12, -12, 4.0);
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);


    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = FL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = FR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            FL.setTargetPosition(newLeftTarget);
            FR.setTargetPosition(newRightTarget);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FL.setPower(Math.abs(speed));
            FR.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && FR.isBusy())) {
                telemetry.addData("Running to", "%7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", "%7d :%7d",
                        FL.getCurrentPosition(), FR.getCurrentPosition());
                telemetry.update();

            }
            FL.setPower(0);
            FR.setPower(0);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }

    }
}