package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ftc16072.MecanumDrive;
import virtual_robot.util.AngleUtils;

@TeleOp(name = "Dpad Mech", group = "Mechanum")
public class DPadMech extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;

    // Code to run ONCE when the driver hits INIT
   /* @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
    } */

    DcMotor m1, m2, m3, m4;

    public void runOpMode(){

        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Press Start to Continue","");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.dpad_up) {
                m1.setPower(0);
                m2.setPower(.5);
                m3.setPower(0.5);
                m4.setPower(0);
            } if(gamepad1.dpad_down) {
                m1.setPower(0);
                m2.setPower(-.5);
                m3.setPower(-.5);
                m4.setPower(0);
            } if(gamepad1.dpad_right) {
                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(.5);
                m4.setPower(.5);
            } if (gamepad1.dpad_left) {
                m1.setPower(.5);
                m2.setPower(.5);
            } else {
                m1.setPower(0);
                m2.setPower(0);
            }
            m3.setPower(0);
            m4.setPower(0);

        }

    }
}


