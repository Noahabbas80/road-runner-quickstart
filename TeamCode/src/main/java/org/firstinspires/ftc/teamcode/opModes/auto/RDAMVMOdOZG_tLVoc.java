package org.firstinspires.ftc.teamcode.opModes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;

@Autonomous(name="simpleandclean_anextenstionofthecolangelofiles")
public class RDAMVMOdOZG_tLVoc extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        Vector2d botOffset = new Vector2d(16.7717/2,15.3543/2);
        Pose2d startPos = new Pose2d(-24-botOffset.x,-24-botOffset.y,Math.toRadians(270));
        Vector2d firePos = new Vector2d(-24-botOffset.x,-24-botOffset.y);

        intake intake = new intake();
        shooter shooter = new shooter();
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        class intakeOn implements Action {
            ElapsedTime timer;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setIntake(1);
                return false;
            }
        }

        class intakeOff implements Action {
            ElapsedTime timer;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setIntake(.25);
                return false;
            }
        }

        class fire implements Action {
            ElapsedTime timer;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
              shooter.fire();
              sleep(1500);
              return false;
            }
        }
        class start implements Action {
            ElapsedTime timer;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooter.start(696969690);
                sleep(3000);
                return false;
            }
        }

        while (!isStarted()) {


            waitForStart();

            TrajectoryActionBuilder collectFront = drive.actionBuilder(startPos)
                    .strafeTo(new Vector2d(-12,-24-botOffset.y))
                    .strafeTo(new Vector2d(-12,-65+botOffset.y))
                    .setTangent(45)
                    .splineToConstantHeading(new Vector2d(-2,-55), Math.toRadians(270))
                    .setTangent(90)
                    .splineToConstantHeading(firePos, Math.toRadians(135));


            Action intakeOn = new intakeOn();
            Action intakeOff = new intakeOff();
            Action fire = new fire();
            Action start = new start();

            Actions.runBlocking(
                    new ParallelAction(

                    new SequentialAction(
                            start,
                            fire,
                            intakeOn,
                            collectFront.build(),
                            fire
                    ))
                    );
        }
    }
}
