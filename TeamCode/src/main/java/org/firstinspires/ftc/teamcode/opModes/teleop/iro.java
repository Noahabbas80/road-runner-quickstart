package org.firstinspires.ftc.teamcode.opModes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter;

@TeleOp(name = "iro")

public class iro extends LinearOpMode {
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    private Servo turretServo;
    DcMotor intake;
    drivetrain drive = new drivetrain();
    shooter shooter = new shooter();
    @Override

    public void runOpMode() throws InterruptedException {

        waitForStart();
        drive.init(hardwareMap);
        shooter.init(hardwareMap,true);
        intake = hardwareMap.dcMotor.get("intake");

        turretServo.setPosition(gamepad1.right_stick_y*.5 + .5);
        while (opModeIsActive()) {
            drive.move(gamepad1);
            intake.setPower(gamepad1.left_bumper ? 1 : gamepad1.square ? -1 : .22);

            if(!previousGamepad1.right_bumper && currentGamepad1.right_bumper){
                shooter.fire();
            }
            telemetry.addData("turret",turretServo.getPosition());
            telemetry.update();
        }
    }



}