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
        hoodLut = new LUT();
        hoodLut.add(50, 0.7);
        hoodLut.add(60, 0.55);
        hoodLut.add(70, 0.45);
        hoodLut.add(80, 0.4);
        hoodLut.add(90, 0.35);
        hoodLut.add(100, 0.3);
        hoodLut.add(110, 0.3);
        hoodLut.add(120, 0.3);
        hoodLut.add(132, 0);
        hoodLut.add(137, 0);
        hoodLut.add(142, 0);
        hoodLut.add(147, 0);
        hoodLut.add(153, 0);
        hoodLut.add(158, 0);
        hoodLut.add(165, 0);



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