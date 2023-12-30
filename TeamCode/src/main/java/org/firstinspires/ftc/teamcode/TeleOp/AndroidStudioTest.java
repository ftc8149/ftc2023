package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="TeleOp Program", group="Drive")
public class AndroidStudioTest extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor RR = hardwareMap.get(DcMotor.class, "RR");
        DcMotor RL = hardwareMap.get(DcMotor.class, "RL");

        DcMotor Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo Pin_Release = hardwareMap.get(Servo.class, "Pin_Release");
        Servo Drone_Trigger = hardwareMap.get(Servo.class, "Drone_Trigger");
        Servo Drone_Pivot = hardwareMap.get(Servo.class, "Drone_Pivot");
        //Servo Dropper = hardwareMap.get(Servo.class, "Dropper");

        FL.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);

        int initialPosition;
        int outtakeTarget;
        boolean isMotorOut = false;

        initialPosition = Outtake.getCurrentPosition() + 10;
        outtakeTarget = initialPosition + 1000;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            double drive = -gamepad1.left_stick_y;
            //axial is the forward and backward motion of the robot
            double turn = gamepad1.right_stick_x;
            //lateral is the turning left/right of the robot
            double strafe = gamepad1.left_stick_x * 0.5;
            //yaw is the left and right/strafing motion of the robot

            double FLPower = drive + turn + strafe;
            double FRPower = drive - turn - strafe;
            double RRPower = drive - turn + strafe;
            double RLPower = drive + turn - strafe;

            double frontMax = Math.max(Math.abs(FLPower), Math.abs(FRPower));
            double backMax = Math.max(Math.abs(RLPower), Math.abs(RRPower));
            double max = Math.max(frontMax, backMax);

            if (max > 1.0) {
                FLPower /= max;
                FRPower /= max;
                RLPower /= max;
                RRPower /= max;
            }

            if (gamepad1.right_bumper){
                FL.setPower(FLPower * 0.5);
                RL.setPower(RLPower * 0.5);
                FR.setPower(RLPower * 0.5);
                RR.setPower(RRPower * 0.5);
            }
            else {
                FL.setPower(FLPower);
                RL.setPower(RLPower);
                FR.setPower(FRPower);
                RR.setPower(RRPower);
            }

            if(gamepad1.a) {
                Pin_Release.setPosition(1.0);
            }
            else {
                Pin_Release.setPosition(0.0);
            }

            if (gamepad2.a) {
                if (!isMotorOut) {
                    // Move the motor out
                    Outtake.setTargetPosition(outtakeTarget);
                    Outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Outtake.setPower(0.5); // Adjust power as needed
                    isMotorOut = true;
                } else {
                    // Move the motor back
                    Outtake.setTargetPosition(initialPosition);
                    Outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Outtake.setPower(0.5); // Adjust power as needed
                    isMotorOut = false;
                }

                Outtake.setPower(0);
                Outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("Outtake position", Math.abs(Outtake.getCurrentPosition()));




            //gamepad1.dpad_up release pin
            //gamepad1.a pull in
            //gamepad1.b push out

            //gamepad2.dpad_up drone_pivot load
            //gamepad2.dpad_right drone_trigger
            //gamepad2.dpad_down drone_pivot reset
            //gamepad2.left_stick_y manual lift
            //gamepad2.x dropper_servo
            //gamepad2.a lift out
            //gamepad2.b lift in



            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Slow Mode", gamepad1.right_bumper);
            telemetry.update();
        }

    }
}
