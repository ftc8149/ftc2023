package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="Single Motor Tester", group="Test")
public class SingleMotorTester extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        boolean hatPressed = false;
        int direction = 1;

        DcMotor motor = hardwareMap.get(DcMotor.class, "Clippy");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double amplifier = 0;
        while (opModeIsActive()) {
            telemetry.addData("Press A", "Full Power");
            telemetry.addData("Press B", "Half Power");
            telemetry.addData("Press X", "Quarter Power");
            telemetry.addData("Press Y", "Three-Quarters Power");

            telemetry.addData("", "");

            telemetry.addData("Amplifier", amplifier);
            telemetry.addData("DPad Down", "Amp: -0.2");
            telemetry.addData("DPad Up", "Amp: 0.2;");
            telemetry.addData("DPad Right", "Amp: 0.3");
            telemetry.addData("DPad Left", "Amp: -0.3");

            telemetry.addData("Direction", direction);
            telemetry.addData("Right Bumper", "Dir: 1");
            telemetry.addData("Left Bumper", "Dir: -1");

            telemetry.update();
            if (gamepad1.a){
               motor.setPower(direction * (1 + amplifier));
            } else if (gamepad1.b){
                motor.setPower(direction * (0.5 + amplifier));
            } else if (gamepad1.x){
                motor.setPower(direction * (0.25 + amplifier));
            } else if (gamepad1.y){
                motor.setPower(direction * (0.75 + amplifier));
            } else {
                motor.setPower(0);
            }

            if (gamepad1.dpad_down || gamepad1.dpad_up) {
                if (!hatPressed) {
                    hatPressed = true;
                    if (gamepad1.dpad_down) {
                        amplifier -= 0.2;
                    } else if (gamepad1.dpad_up) {
                        amplifier += 0.2;
                    }
                }
            } else {
                hatPressed = false;
            }

            if(gamepad1.start){
                amplifier = 0;
            }

            if (gamepad1.right_bumper) {
                direction = 1;
            } else if (gamepad1.left_bumper) {
                direction = -1;
            }
        }


    }

}
