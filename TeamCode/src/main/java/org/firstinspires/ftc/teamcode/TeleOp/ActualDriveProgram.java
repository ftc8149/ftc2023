package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware. Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="yes7 DRIVE PROGRAM", group="Drive")
public class ActualDriveProgram extends LinearOpMode{

    DcMotor FL, FR, RR, RL, Outtake, Clippy, Intake;
    Servo Pin_Release, Drone_Trigger, Drone_Pivot, Dropper;
    boolean dropperActivated = false;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime dropperElapsedTime = new ElapsedTime();
    double slow = 1.0;
    double direction = 1.0;


    @Override
    public void runOpMode() {

        initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            FL.setPower(((drive + strafe + turn) * slow) * direction);
            FR.setPower(((drive - strafe - turn) * slow) * direction);
            RL.setPower(((drive - strafe + turn) * slow) * direction);
            RR.setPower(((drive + strafe - turn) * slow) * direction);

            if(gamepad1.a){
                Clippy.setPower(1);
            } else if(gamepad1.b){
                Clippy.setPower(-1);
            } else{
                Clippy.setPower(0);
            }

            if(gamepad1.right_bumper){
                slow = 0.5;
            } else if(gamepad1.left_bumper){
                slow = 1.0;
            }


            if(gamepad1.dpad_down){
                Pin_Release.setPosition(1.0);
            } else if(gamepad1.dpad_up){
                Pin_Release.setPosition(0.0);
            }

            if (gamepad2.right_bumper) {
                int OUTTAKE_TARGET_POSITION = 3000;
                moveOuttakeToPosition(OUTTAKE_TARGET_POSITION);
            } else if (gamepad2.left_bumper) {
                moveOuttakeToPosition(30);
            } else {
                Outtake.setPower(-gamepad2.left_stick_y);
            }

            if(gamepad2.dpad_up){
                Drone_Pivot.setPosition(0.5);
            } else if(gamepad2.dpad_down){
                Drone_Pivot.setPosition(1.0);
            }

            if(gamepad2.dpad_right){
                Drone_Trigger.setPosition(0.4);
            } else if(gamepad2.dpad_left){
                Drone_Trigger.setPosition(0.0);
            }

            if(gamepad2.start){
                Drone_Pivot.setPosition(1.0);
                Drone_Trigger.setPosition(0.0);
            }

            if(gamepad2.x){
                dropperElapsedTime.reset();
                Dropper.setPosition(0.0);
                dropperActivated = true;
            }
            if(dropperElapsedTime.seconds() >= 1){
                Dropper.setPosition(1.0);
                dropperActivated = false;
            }

            if(gamepad2.b){
                Intake.setPower(1);
            } else if(gamepad2.start){
                Intake.setPower(-1);
            } else {
                Intake.setPower(0);
            }

            if (gamepad1.start) {
                direction = -1.0; // Flip the direction
            } else if (gamepad1.back){
                direction = 1.0;
            }




            telemetry.addData("FL Power", FL.getPower());
            telemetry.addData("FR Power", FR.getPower());
            telemetry.addData("RR Power", RR.getPower());
            telemetry.addData("RL Power", RL.getPower());
            telemetry.addLine();
            telemetry.addData("Direction", direction);
            telemetry.addLine();
            telemetry.addData("Pivot Position", Drone_Pivot.getPosition());
            telemetry.addData("Trigger Position", Drone_Trigger.getPosition());
            telemetry.addData("Pin Release", Pin_Release.getPosition());
            telemetry.addLine();
            telemetry.addData("Outtake Position", Outtake.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Dropper Activation?", dropperActivated);
            telemetry.addData("Dropper Position", Dropper.getPosition());
            telemetry.addLine();
            telemetry.addData("Pivot Position", Drone_Pivot.getPosition());
            telemetry.addLine();
            telemetry.addData("Runtime", runtime);
            telemetry.addData("Dropper Elapsed Time", dropperElapsedTime);
            telemetry.update();

        }

    }

    private void moveOuttakeToPosition(int targetPosition) {
        Outtake.setTargetPosition(targetPosition);

        boolean outtakeMoving = false;
        if(targetPosition == 30){
            Outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Outtake.setPower(-0.25);
            outtakeMoving = true;
        } else {
            Outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Outtake.setPower(0.25);
            outtakeMoving = true;
        }

        while (Outtake.isBusy() && opModeIsActive()) {

            if (targetPosition == 30) {
                if (Math.abs(Outtake.getCurrentPosition()) <= (Math.abs(targetPosition) + 20)) {
                    Outtake.setPower(0);
                    Outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outtakeMoving = false;
                }
            } else {
                if (Math.abs(Outtake.getCurrentPosition()) >= targetPosition) {
                    Outtake.setPower(0);
                    Outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outtakeMoving = false;
                }
            }
        }




    }

    private void initialize(){

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RR = hardwareMap.get(DcMotor.class, "RR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Clippy = hardwareMap.get(DcMotor.class, "Clippy");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        Pin_Release = hardwareMap.get(Servo.class, "Pin_Release");
        Pin_Release.scaleRange(0.0, 1.0);

        Drone_Trigger = hardwareMap.get(Servo.class, "Drone_Trigger");
        Drone_Trigger.scaleRange(0.0, 0.4);

        Drone_Pivot = hardwareMap.get(Servo.class, "Drone_Pivot");
        Drone_Pivot.scaleRange(0.0, 1.0);

        Dropper = hardwareMap.get(Servo.class, "Dropper");
        Dropper.scaleRange(0.0,1.0);

        FL.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);
        Clippy.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Pin_Release.setDirection(Servo.Direction.REVERSE);
        Drone_Trigger.setDirection(Servo.Direction.FORWARD);
        Drone_Pivot.setDirection(Servo.Direction.FORWARD);
        Dropper.setDirection(Servo.Direction.REVERSE);

        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}