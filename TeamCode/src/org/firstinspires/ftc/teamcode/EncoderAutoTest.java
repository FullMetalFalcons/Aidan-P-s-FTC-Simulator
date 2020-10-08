package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import virtual_robot.util.AngleUtils;

@Autonomous(name = "Encoder Auto", group = "FMF")
public class EncoderAutoTest extends LinearOpMode {


    private final double ENCODER_TICKS_PER_REVOLUTION = 1120;


    DcMotorEx left = (DcMotorEx) hardwareMap.dcMotor.get("left_motor");
    DcMotorEx right = (DcMotorEx) hardwareMap.dcMotor.get("right_motor");


    private void stopDpad(){
        left.setPower(0);
        right.setPower(0);
    }
    private void moveForward() {
        left.setPower(.5);
        right.setPower(.5);
    }
    private void moveBackward() {
        left.setPower(-.5);
        right.setPower(-.5);
    }
    private void turnRight() {
        left.setPower(.5);
        right.setPower(-.5);
    }
    private void turnLeft() {
        left.setPower(-0.5);
        right.setPower(0.5);
    }
    private void resetEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    @Override
    public void runOpMode(){

        double leftPos = left.getCurrentPosition();
        double rightPos = right.getCurrentPosition();



        left.setDirection(DcMotor.Direction.REVERSE);
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        Servo backServo = hardwareMap.servo.get("back_servo");
        gyro.init();
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        double currentHeading = gyro.getHeading();

        telemetry.addData("Press Start to Continue", " Running autonomous code");
        telemetry.update();

        while (left.getCurrentPosition() <= 4200) {
            moveForward();
        }
        resetEncoders();
        while (left.getCurrentPosition() >= -1112 && right.getCurrentPosition() <= 1113) {
            //The reason for 1112 is because the left encoder is 8 ticks to high, so this limits it to a 90 degree turn.
            //As for 1113 on the right, it is because the right encoder is 7 ticks to high.
            turnLeft();
        }
        resetEncoders();
        while (left.getCurrentPosition() <= 1120) {
            moveForward();
        }
        sleep(100);
        resetEncoders();
        while (left.getCurrentPosition() >= -1120){
            moveBackward();
        }
        resetEncoders();
        while (left.getCurrentPosition() >= -1112 && right.getCurrentPosition() <= 1113) {
            turnLeft();
        }
        resetEncoders();
        while (left.getCurrentPosition() <= 4200) {
            moveForward();
        }
        stopDpad();

        telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
        telemetry.update();

        //All auto before here
        waitForStart();
        //All teleop after here
        gyro.getHeading();
        telemetry.addData("Press Stop to End", "Running Teleop");
        telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
        telemetry.update();
        while(opModeIsActive()) {
           if (gamepad1.dpad_up) {
               moveForward();
           } else if (gamepad1.dpad_down) {
               moveBackward();
           } else {
               stopDpad();
           }


        }
stopDpad();
    }


}
