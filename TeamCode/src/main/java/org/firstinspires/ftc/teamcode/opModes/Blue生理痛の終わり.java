package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.vision;
@TeleOp(name = "Blue生理痛の終わり")

public class Blue生理痛の終わり extends LinearOpMode {
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    boolean ended = false;
    public ElapsedTime runTime = new ElapsedTime();

    drivetrain drive = new drivetrain();
    shooter shooter = new shooter();
    intake intake = new intake();
    vision vision = new vision();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        shooter.init(hardwareMap,telemetry);
        intake.init(hardwareMap);
        vision.init(hardwareMap);

        int nessID = hardwareMap.appContext.getResources().getIdentifier("ness", "raw", hardwareMap.appContext.getPackageName());

        waitForStart();
        runTime.reset();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            shooter.align(vision.offset(telemetry,20),telemetry);
            intake.setIntake(gamepad1.left_bumper ? 1 : gamepad1.square ? -1 : .25);
            if(!previousGamepad1.right_bumper && currentGamepad1.right_bumper){
                shooter.fire(1);
            }
            telemetry.addData("time",runTime.milliseconds()/1000);
            if (runTime.seconds() > 123 && !ended)
            {
                ended = true;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, nessID);
            }
            telemetry.update();
        }
    }



}