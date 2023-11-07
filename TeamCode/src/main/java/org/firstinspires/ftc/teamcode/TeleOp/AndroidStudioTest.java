package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp Program", group="LinearOpMode")
public class AndroidStudioTest extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor RR = null;
    private DcMotor RL = null;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "LF");
        FR = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");
        RL = hardwareMap.get(DcMotor.class, "LR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            double max;

            double axial = -gamepad1.left_stick_y;
            //axial is the forward and backward motion of the robot
            double lateral = gamepad1.left_stick_x;
            //lateral is the left and right/strafing motion of the robot
            double yaw = gamepad1.right_stick_x;
            //yaw is the turning/rotation of the robot

            double FLPower = axial + lateral + yaw;
            double FRPower = axial - lateral - yaw;
            double RRPower = axial - lateral + yaw;
            double RLPower = axial + lateral - yaw;

            max = Math.max(Math.abs(FLPower), Math.abs(FRPower));
            max = Math.max(max, Math.abs(RLPower));
            max = Math.max(max, Math.abs(RRPower));

            if (max > 1.0); {
                FLPower /= max;
                FRPower /= max;
                RLPower /= max;
                RRPower /= max;
            }

            FL.setPower(FLPower);
            RL.setPower(RLPower);
            FR.setPower(FRPower);
            RR.setPower(RRPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2, %4.2", FLPower, FRPower);
            telemetry.addData("Back left/Right", "%4.2, %4.2", FLPower, FRPower);
            telemetry.update();
        }

    }
}
