package org.firstinspires.ftc.teamcode.opModes.auto;
import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="simpleandclean_anextenstionofthecolangelofiles")
public class simpleandclean_anextenstionofthecolangelofiles extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        Vector2d botOffset = new Vector2d(16.7717/2,15.3543/2);
        Pose2d startPos = new Pose2d(72-botOffset.x,-24+botOffset.y,Math.toRadians(270));
        Vector2d firePos = new Vector2d(69-16.7717/2,-27+15.3543/2);
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
                shooter.start(1);
                sleep(3000);
                return false;
            }
        }

        while (!isStarted()) {


            waitForStart();

            TrajectoryActionBuilder collectMiddle = drive.actionBuilder(startPos)
                    .strafeTo(new Vector2d(12,-24+botOffset.y))
                    .strafeTo(new Vector2d(12,-72+botOffset.y));

            TrajectoryActionBuilder middleToFire = drive.actionBuilder(new Pose2d(12,-72+botOffset.y,270))
                    .setTangent(90)
                    .splineToConstantHeading(new Vector2d(0,-55), Math.toRadians(180))
                    .strafeTo(new Vector2d(0,-60))
                    .setTangent(45)
                    .splineToConstantHeading(firePos, Math.toRadians(0));

            TrajectoryActionBuilder collectBack = drive.actionBuilder(new Pose2d(firePos,270))
                    .strafeTo(new Vector2d(36,-27+botOffset.y))
                    .strafeTo(new Vector2d(36,-72+botOffset.y));

            TrajectoryActionBuilder backToFire = drive.actionBuilder(new Pose2d(new Vector2d(36,-72+botOffset.y),270))
                    .setTangent(45)
                    .splineToConstantHeading(firePos, Math.toRadians(45));

            TrajectoryActionBuilder collectFront = drive.actionBuilder(new Pose2d(firePos,270))
                    .strafeTo(new Vector2d(-12,-27+botOffset.y))
                    .strafeTo(new Vector2d(-12,-65+botOffset.y));

            TrajectoryActionBuilder frontToFire = drive.actionBuilder(new Pose2d(new Vector2d(-12,-65+botOffset.y),270))
                    .setTangent(45)
                    .splineToConstantHeading(firePos, Math.toRadians(0));

            TrajectoryActionBuilder end = drive.actionBuilder(new Pose2d(firePos,270))
                    .strafeTo(new Vector2d(12,-72+botOffset.y));



            Action intakeOn = new intakeOn();
            Action intakeOff = new intakeOff();
            Action fire = new fire();
            Action start = new start();

            Actions.runBlocking(
                    new SequentialAction(
                            start,
                            fire,
                            intakeOn,
                            collectMiddle.build(),
                            intakeOff,
                            middleToFire.build(),
                            fire,
                            intakeOn,
                            collectBack.build(),
                            intakeOff,
                            backToFire.build(),
                            fire,
                            intakeOn,
                            collectFront.build(),
                            intakeOff,
                            frontToFire.build(),
                            fire,
                            end.build()
                    ));
        }
    }
}
