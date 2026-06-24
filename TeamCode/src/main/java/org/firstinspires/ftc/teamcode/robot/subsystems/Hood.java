package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.utils.LUT;

public class Hood {
    private Servo hoodServo;

    // Отдельная таблица для худа (Дистанция -> Позиция сервопривода)
    public LUT hoodLut;

    public void init(HardwareMap hw) {
        // Убедись, что в конфигурации телефона серва называется "hood"
        hoodServo = hw.get(Servo.class, "hood");

        hoodLut = new LUT();

        // ПРИМЕР ЗНАЧЕНИЙ (тебе нужно будет найти свои):
        // 0.0 - капюшон максимально опущен (стреляем высоко вверх)
        // 1.0 - капюшон максимально поднят (стреляем прямо перед собой)
        hoodLut.add(43.6, 0.25);
        hoodLut.add(75.8, 0.40);
        hoodLut.add(102.6, 0.55);
        hoodLut.add(135.6, 0.65);
        hoodLut.add(150.6, 0.75);
    }

    public void setPosition(double pos) {
        // Опционально: можно добавить Range.clip(pos, min, max),
        // чтобы случайно не выломать пластик худа, если серва уйдет слишком далеко
        hoodServo.setPosition(pos);
    }

    /**
     * Автоматически выставляет угол худа по расстоянию до цели
     */
    public void setDistance(double targetDistance) {
        setPosition(hoodLut.get(targetDistance));
    }

    public double getPosition() {
        return hoodServo.getPosition();
    }

    public void periodic() {
        // Для сервы обычно пусто
    }
}