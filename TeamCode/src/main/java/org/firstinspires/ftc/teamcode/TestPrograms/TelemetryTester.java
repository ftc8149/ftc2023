package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp Tester", group="Test")
@Disabled
public class TelemetryTester extends LinearOpMode{

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            double max;

            double drive = -gamepad1.left_stick_y;
            //axial is the forward and backward motion of the robot
            double turn = gamepad1.right_stick_x;
            //lateral is the turning left/right of the robot
            double strafe = gamepad1.left_stick_x;
            //yaw is the left and right/strafing motion of the robot

            double FLPower = drive + turn + strafe;
            double FRPower = drive - turn - strafe;
            double RRPower = drive - turn + strafe;
            double RLPower = drive + turn - strafe;

            max = Math.max(Math.abs(FLPower), Math.abs(FRPower));

            if (max > 1.0) {
                FLPower /= max;
                FRPower /= max;
                RLPower /= max;
                RRPower /= max;
            }

            double FL = (FLPower);
            double RL = (RLPower);
            double FR = (FRPower);
            double RR = (RRPower);

            if (gamepad1.right_bumper){

                FL = (FLPower * 0.5);
                RL = (RLPower * 0.5);
                FR = (FRPower * 0.5);
                RR = (RRPower * 0.5);
            }

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("FL Speed", FL);
            telemetry.addData("RL Speed", RL);
            telemetry.addData("FR Speed", FR);
            telemetry.addData("RR Speed", RR);
            telemetry.addData("Slow Mode", gamepad1.right_bumper);
            telemetry.update();
        }

    }
}
