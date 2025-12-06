package org.firstinspires.ftc.teamcode.opModes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="redAuto")
public class redAuto extends LinearOpMode {
    private DcMotorEx shooter1,shooter2,intake;
    private Servo turretServo,latchServo,rampServo;



    public void runOpMode() throws InterruptedException {
        Vector2d botOffset = new Vector2d(16.7717/2,15.3543/2);
        Pose2d startPos = new Pose2d(-24-botOffset.x,-24+botOffset.y,Math.toRadians(90));
        Vector2d firePos = new Vector2d(-24-botOffset.x,-24+botOffset.y);


        rampServo = hardwareMap.servo.get("rampServo");
        latchServo = hardwareMap.servo.get("latchServo");
        turretServo = hardwareMap.servo.get("turretServo");

        shooter1 = (DcMotorEx) hardwareMap.dcMotor.get("shooter1");
        shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);


        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rampServo.setPosition(.01);
        latchServo.setPosition(.26);
        turretServo.setPosition(.2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        class fire implements Action {
            ElapsedTime timer;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                    latchServo.setPosition(.49);
                    sleep(1500);
                latchServo.setPosition(.26);
              return false;
            }
        }


        class start implements Action {
            ElapsedTime timer;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooter1.setVelocity(1325);
                shooter2.setVelocity(1325);
                intake.setPower(1);
                return (shooter1.getVelocity() < 1325 && shooter2.getVelocity() <1325);
            }
        }


            waitForStart();

            TrajectoryActionBuilder collectFront = drive.actionBuilder(startPos)
                    .strafeTo(new Vector2d(-12,24-botOffset.y))
                    .strafeTo(new Vector2d(-12,65-botOffset.y))
//                    .setTangent(45)
//                    .splineToConstantHeading(new Vector2d(-2,-55), Math.toRadians(270))
                    .setTangent(180)
                    .splineToConstantHeading(firePos, Math.toRadians(225));

        TrajectoryActionBuilder collectMiddle = drive.actionBuilder(startPos)
                .strafeTo(new Vector2d(12,24-botOffset.y))
                .strafeTo(new Vector2d(12,65-botOffset.y))
                .setTangent(180)
                .splineToConstantHeading(firePos, Math.toRadians(225));

        TrajectoryActionBuilder collectBack = drive.actionBuilder(startPos)
                .strafeTo(new Vector2d(36,24-botOffset.y))
                .strafeTo(new Vector2d(36,65-botOffset.y))
                .setTangent(180)
                .splineToConstantHeading(firePos, Math.toRadians(225));


        TrajectoryActionBuilder leave = drive.actionBuilder(startPos)
                .strafeTo(new Vector2d(-12,40));


            Action fire = new fire();
            Action start = new start();

            Actions.runBlocking(
                    new SequentialAction(
                            start,
                            fire,
                            collectFront.build(),
                            fire,
                            collectMiddle.build(),
                            fire,
                            collectBack.build(),
                            fire,
                            leave.build()
                  )
                    );

    }
}
