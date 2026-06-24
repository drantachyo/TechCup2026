package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Stopper {
    private Servo stopperServo;

    // Настрой эти позиции в Panels под вашу механику!
    public static double OPEN_POS = 0.6;   // Позиция: проход открыт (для выстрела)
    public static double CLOSED_POS = 0.2; // Позиция: проход закрыт (для езды и забора)

    public void init(HardwareMap hw) {
        // Убедитесь, что в конфигурации телефона серва называется "stopper"
        stopperServo = hw.get(Servo.class, "stopper");

        // При включении робота заслонка ВСЕГДА должна быть закрыта
        close();
    }

    public void open() {
        stopperServo.setPosition(OPEN_POS);
    }

    public void close() {
        stopperServo.setPosition(CLOSED_POS);
    }

    public void periodic() {
        // Пусто
    }
}