package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;


@Autonomous(name = "Test Auto", group = "FMF")
public class AutoTest extends LinearOpMode {

    DcMotorEx left = (DcMotorEx) hardwareMap.dcMotor.get("left_motor");
    DcMotorEx right = (DcMotorEx) hardwareMap.dcMotor.get("right_motor");


    private void stopDpad(){
        left.setPower(0);
        right.setPower(0);
    }

    private void turnRight() {
        left.setPower(.5);
        right.setPower(-.5);
    }

    private void turnLeft() {
        left.setPower(-0.5);
        right.setPower(0.5);
    }

    private void moveForward() {
        left.setPower(.5);
        right.setPower(.5);
    }

    private void moveBackward() {
        left.setPower(-.5);
        right.setPower(-.5);
    }

    private void toPlatformReturn() {
        turnLeft();
        sleep(890);
        moveForward();
        sleep(1000);
        moveBackward();
        sleep(1000);
        turnLeft();
        sleep(890);
    }

    int i;
    private void accelForward() {
        while (i<5) {
           left.setPower(.2 + (i*.1));
           right.setPower(.2+(i*.1));
           sleep(250 + (i*250));
           i++;
        }
        stopDpad();
    }


    @Override
    public void runOpMode() throws InterruptedException {

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


       // accelForward();
       // sleep(500);
       // Use encoders now and figure out number of ticks to figure out positioning of the robot and check in code on GitHub

        moveForward();
        sleep(3500);
        toPlatformReturn();
        moveForward();
        sleep(6000);
        turnRight();
        sleep(890);
        moveForward();
        sleep(1000);
        moveBackward();
        sleep(1000);
        turnRight();
        sleep(890);
        moveForward();
        sleep(6000);
        toPlatformReturn();
        moveForward();
        sleep(3000);
        stopDpad();

        //All auto before here
        waitForStart();
        //All teleop after here
        gyro.getHeading();
        telemetry.addData("Press Stop to End", "Running Teleop");
        telemetry.update();

    }
}
