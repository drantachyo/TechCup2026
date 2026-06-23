package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intakeMotor;

    public void init(HardwareMap hw) {
        // Инициализируем мотор из конфигурации на телефоне
        intakeMotor = hw.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void periodic() {
        // Для интейка тут обычно пусто, но метод обязан быть для структуры
    }
}