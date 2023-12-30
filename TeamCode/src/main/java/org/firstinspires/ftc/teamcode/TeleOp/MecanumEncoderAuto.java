package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Encoder Auto", group="Robot")
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

        //telemetry.addData("Front starting at", "%7d :%7d",
                //FL.getCurrentPosition(),
                //FR.getCurrentPosition());
       // telemetry.addData("Back starting at", "%7d :%7d",
                //RR.getCurrentPosition(),
                //.getCurrentPosition());

        telemetry.addData("Starting at", "7%d :7%d :7%d :7%d",
                FL.getCurrentPosition(),
                FR.getCurrentPosition(),
                RL.getCurrentPosition(),
                RR.getCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED, 48, 48, 48,48,5.0);
        encoderDrive(TURN_SPEED, 12, -12, 12,-12,4.0);
        encoderDrive(DRIVE_SPEED, -24, -24, -24,-24,4.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);


    }

    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,
                             double rearLeftInches, double rearRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        if (opModeIsActive()) {

            newFrontLeftTarget = FL.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = FR.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newRearLeftTarget = RL.getCurrentPosition() + (int) (rearLeftInches * COUNTS_PER_INCH);
            newRearRightTarget = RR.getCurrentPosition() + (int) (rearRightInches * COUNTS_PER_INCH);
            FL.setTargetPosition(newFrontLeftTarget);
            FR.setTargetPosition(newFrontRightTarget);
            RR.setTargetPosition(newRearRightTarget);
            RL.setTargetPosition(newRearLeftTarget);

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
                //telemetry.addData("Running to", "%7d :%7d", newLeftTarget, newRightTarget);
                //telemetry.addData("Running to", "%7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                telemetry.addData("FL Running to", newFrontLeftTarget);
                telemetry.addData("FR Running to", newFrontRightTarget);
                telemetry.addData("RL Running to", newRearLeftTarget);
                telemetry.addData("RR Running to", newRearRightTarget);
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
