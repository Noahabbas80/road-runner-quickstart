package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class shooter {
    private PIDController controller;
    private DcMotor shooter1,shooter2;
    private Servo turretServo;
    public void init(HardwareMap hwm){
        shooter1 = hwm.dcMotor.get("shooter1");
        shooter2 = hwm.dcMotor.get("shooter2");
        turretServo = hwm.servo.get("turretServo");

        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void fire(Gamepad gamepad){


    }
}
