package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.intake;
@TeleOp(name = "lockin")

public class lockin extends LinearOpMode {
    intake intake = new intake();

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
           intake.setIntake(1);

        }

    }
}

