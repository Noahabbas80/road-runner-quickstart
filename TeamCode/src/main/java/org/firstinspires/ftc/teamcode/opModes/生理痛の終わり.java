package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
@TeleOp(name = "生理痛の終わり")


public class 生理痛の終わり extends LinearOpMode {
    public enum RobotState {
        GOONING,
        MOGGING,
    };

    RobotState robotState = RobotState.MOGGING;

    DcMotor intake, shooter1 ,shooter2;
    Servo latchServo, turretServo;
    public Gamepad currentGamepad2 = new Gamepad();

    boolean ended = false;
    public Gamepad previousGamepad2 = new Gamepad();
    public ElapsedTime runTime = new ElapsedTime();

    drivetrain drive = new drivetrain();
    shooter shooter = new shooter();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        shooter.init(hardwareMap,telemetry);
        int nessID = hardwareMap.appContext.getResources().getIdentifier("ness", "raw", hardwareMap.appContext.getPackageName());

        intake = hardwareMap.dcMotor.get("intake");

        latchServo = hardwareMap.servo.get("latchServo");


//        rightFront.setDirection(DcMotor.Direction.REVERSE);

        latchServo.setPosition(0);

        waitForStart();
        runTime.reset();

        while (opModeIsActive()) {


            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            drive.move(gamepad1);
            telem();
            if (runTime.seconds() > 123 && !ended)
            {
                ended = true;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, nessID);
            }

        }
    }
    public void telem() {


        telemetry.addData("latch", latchServo.getPosition());

        telemetry.addData("current state", robotState);
        telemetry.update();
    }


}