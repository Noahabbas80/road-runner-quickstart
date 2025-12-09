package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Drive")
public class Drive extends LinearOpMode {
    private DcMotorEx shooter1,shooter2;
    private DcMotor intake,rightFront, rightBack, leftFront, leftBack;;

    private Servo latchServo,rampServo,turretServo;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = (DcMotorEx) hardwareMap.dcMotor.get("shooter1");
        shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");
        intake = hardwareMap.dcMotor.get("intake");
        turretServo = hardwareMap.servo.get("turretServo");
        latchServo = hardwareMap.servo.get("latchServo");
        rampServo = hardwareMap.servo.get("rampServo");

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rampServo.setPosition(1);

        shooter1.setDirection(DcMotor.Direction.REVERSE);


        turretServo.setPosition(.31);

        latchServo.setPosition(.26);
        waitForStart();

        latchServo.setPosition(.26);
        if (opModeIsActive()) {
            shooter1.setVelocity(1250);
            shooter2.setVelocity(1250);
            intake.setPower(1);
            sleep(5000);

            leftFront.setPower(-.295);
            rightFront.setPower(-.295);
            leftBack.setPower(-.3);
            leftFront.setPower(-.3);

            sleep(3000);
            leftFront.setPower(-0);
            rightFront.setPower(-0);
            leftBack.setPower(-0);
            leftFront.setPower(-0);
            latchServo.setPosition(.49);
            sleep(5000);
            latchServo.setPosition(.26);
            leftFront.setPower(-.3);
            rightFront.setPower(.3);
            leftBack.setPower(.3);
            leftFront.setPower(-.3);
            sleep(10000);
        }
    }
}
