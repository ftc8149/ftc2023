package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Mecanum: Encoder Auto", group="Robot")
@Disabled
public class MecanumEncoderAuto extends LinearOpMode {

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor RR = null;
    private DcMotor RL = null;
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
        RR = hardwareMap.get(DcMotor.class, "RR");
        RL = hardwareMap.get(DcMotor.class, "RL");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Front starting at", "%7d :%7d",
                FL.getCurrentPosition(),
                FR.getCurrentPosition());
        telemetry.addData("Back starting at", "%7d :%7d",
                RR.getCurrentPosition(),
                RL.getCurrentPosition());
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
            RR.setTargetPosition(newRightTarget);
            RL.setTargetPosition(newLeftTarget);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FL.setPower(Math.abs(speed));
            FR.setPower(Math.abs(speed));
            RR.setPower(Math.abs(speed));
            RL.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && FR.isBusy() && RR.isBusy() && RL.isBusy())) {
                telemetry.addData("Running to", "%7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Front currently at", "%7d :%7d",
                        FL.getCurrentPosition(), FR.getCurrentPosition());
                telemetry.addData("Back currently at", "%7d :%7d",
                        RR.getCurrentPosition(), RL.getCurrentPosition());
                telemetry.update();

            }
            FL.setPower(0);
            FR.setPower(0);
            RR.setPower(0);
            RL.setPower(0);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }

    }
}
