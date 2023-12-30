package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Motor Position Tracker", group="Test")
@Disabled
public class MotorPositionTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor RR = hardwareMap.dcMotor.get("RR");
        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor RL = hardwareMap.dcMotor.get("RL");

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);

        double FR_offset = FR.getCurrentPosition();
        double RR_offset = RR.getCurrentPosition();
        double FL_offset = FL.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            FR_offset = FR.getCurrentPosition();
            RR_offset = RR.getCurrentPosition();
            FL_offset = FL.getCurrentPosition();

            telemetry.addData("FR", FR_offset);
            telemetry.addData("FL", FL_offset);
            telemetry.addData("RR", RR_offset);
            telemetry.addData("RL", RL.getCurrentPosition());
            telemetry.update();

        }
    }
}
