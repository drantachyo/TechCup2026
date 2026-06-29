package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intakeMotor;

    public void init(HardwareMap hw) {
        // Инициализируем мотор из конфигурации на телефоне
        intakeMotor = hw.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public double getVelocity() {
        // Возвращает текущую скорость мотора в тиках в секунду (TPS)
        return intakeMotor.getVelocity();
    }
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void periodic() {
        // Для интейка тут обычно пусто, но метод обязан быть для структуры
    }
    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS); // Укажи свое имя мотора
    }
}