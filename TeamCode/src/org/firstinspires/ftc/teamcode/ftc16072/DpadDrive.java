package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "dpad drive", group = "TwoWheel")
class DPadDrive extends LinearOpMode {


    @Override
    public void runOpMode() {
        boolean moveLeft = gamepad1.dpad_left;
        boolean moveRight = gamepad1.dpad_right;
        boolean forward = gamepad1.dpad_up;
        boolean backward = gamepad1.dpad_down;


        DcMotor left = hardwareMap.dcMotor.get("left_motor");
        DcMotor right = hardwareMap.dcMotor.get("right_motor");



        telemetry.addData("Press Start to Continue", "");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (moveLeft) {
                telemetry.addData("Strafe Left", "");
                left.setPower(.5);
                right.setPower(0);
            }
            if (moveRight) {
                telemetry.addData("Strafe Right", "");
                left.setPower(0);
                right.setPower(.5);
            }
            if (forward) {
                telemetry.addData("Move Forward", "");
                left.setPower(.5);
                right.setPower(.5);
            }
            if (backward) {
                telemetry.addData("Move Backward", "");
                left.setPower(-.5);
                right.setPower(-.5);
            }
        }

    }

    @Override
    public void init() {

    }


    }



/*
    double moveForward = 0;
    double rotate = 0;
    double strafe = 0;
    mecanumDrive.driveMecanum(moveForward, strafe, rotate);

distances = mecanumDrive.getDistanceCm();
*/





