package org.firstinspires.ftc.teamcode.opModes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "justdrive")

public class justdrive extends LinearOpMode {
    DcMotor rightFront, rightBack, leftFront, leftBack;
    Servo latchServo;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        latchServo = hardwareMap.servo.get("latchServo");
//
//        shooter1 = hardwareMap.dcMotor.get("shooter1");
//        shooter2 = hardwareMap.dcMotor.get("shooter2");
//        intake = hardwareMap.dcMotor.get("intake");

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            double y =-gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower((frontLeftPower) * (gamepad1.x ? .5 : 0));
            leftBack.setPower((backLeftPower) * (gamepad1.x ? .5 : 0));
            rightFront.setPower((frontRightPower) * (gamepad1.x ? .5 : 0));
            rightBack.setPower((backRightPower) * (gamepad1.x ? .5 : 0));
//
//            intake.setPower(1);
//            shooter1.setPower(1);
//            shooter2.setPower(1);
            telemetry.addData("y",y);
            telemetry.addData("x",x);
            telemetry.addData("rx",rx);
            }

        }
    }

