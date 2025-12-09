package org.firstinspires.ftc.teamcode.opModes.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter;

@TeleOp(name = "expBlue")

public class expBlue extends LinearOpMode {
    private Limelight3A limelight;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    DcMotor intake;
    drivetrain drive = new drivetrain();
    shooter shooter = new shooter();
    @Override

    public void runOpMode() throws InterruptedException {

        waitForStart();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        limelight.start();

        drive.init(hardwareMap);
        shooter.init(hardwareMap,true);
        intake = hardwareMap.dcMotor.get("intake");


        while (opModeIsActive()) {
            drive.move(gamepad1);
            intake.setPower(gamepad1.left_bumper ? 1 : gamepad1.square ? -1 : .22);

            if(!previousGamepad1.right_bumper && currentGamepad1.right_bumper){
                shooter.fire();
            }

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    shooter.align(result.getTx());
                    telemetry.addData("Tx",result.getTx());
                }
            }
            telemetry.update();
        }
    }



}