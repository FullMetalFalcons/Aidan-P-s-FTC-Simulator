package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;
import virtual_robot.util.AngleUtils;


@Autonomous(name = "DPad Test Auto", group = "Two Wheel")
public abstract class DPadTestAuto extends LinearOpMode {

    DcMotorEx left = (DcMotorEx) hardwareMap.dcMotor.get("left_motor");
    DcMotorEx right = (DcMotorEx) hardwareMap.dcMotor.get("right_motor");

    private void stopDPad() {
        left.setPower(0);
        right.setPower(0);
    }
    private void autoForward() {
        left.setPower(-.5);
        right.setPower(.5);
    }


    public void runAuto()  {

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

        ElapsedTime waitTime = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Seconds since init", "%d. Press start when ready.", (int) waitTime.seconds());
            telemetry.update();
        }


        while (opModeIsActive()) {
            autoForward();
            waitTime.seconds();
            stopDPad();
            
        }
        stopDPad();
    }

    @Override
    protected void sleep(long milliseconds) {
        super.sleep(milliseconds);
    }
}