package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
@TeleOp(name = "endperiodcramps")


public class endperiodcramps extends LinearOpMode {
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
    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        int nessID = hardwareMap.appContext.getResources().getIdentifier("ness", "raw", hardwareMap.appContext.getPackageName());

        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");
        intake = hardwareMap.dcMotor.get("intake");

        turretServo = hardwareMap.servo.get("turretServo");
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