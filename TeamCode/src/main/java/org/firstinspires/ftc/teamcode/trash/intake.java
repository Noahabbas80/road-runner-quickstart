package org.firstinspires.ftc.teamcode.trash;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class intake {
    private DcMotor intake;
    private Servo leverServo1, LeverServo2;
    public void init(HardwareMap hwm){
        intake = hwm.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

// this is the saddest thing ive ever made in my life.
    public void setIntake(double power){
        intake.setPower(power);
    }

}

