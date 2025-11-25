package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drivetrain {
    DcMotor rightFront, rightBack, leftFront, leftBack;

    public void init(HardwareMap hwm){
        leftFront = hwm.dcMotor.get("leftFront");
        rightFront = hwm.dcMotor.get("rightFront");
        leftBack = hwm.dcMotor.get("leftBack");
        rightBack = hwm.dcMotor.get("rightBack");

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void move(Gamepad gamepad){
        double y =-gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = -gamepad.right_stick_x;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower((frontLeftPower) * (gamepad.x ? .5 : 1));
        leftBack.setPower((backLeftPower) * (gamepad.x ? .5 : 1));
        rightFront.setPower((frontRightPower) * (gamepad.x ? .5 : 1));
        rightBack.setPower((backRightPower) * (gamepad.x ? .5 : 1));

    }
}
