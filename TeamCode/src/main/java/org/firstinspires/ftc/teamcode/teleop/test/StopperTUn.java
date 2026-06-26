package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Smooth Stopper Tuner", group = "Tuning")
public class StopperTUn extends LinearOpMode {

    private Servo stopperServo;
    private double currentPosition = 0.5; // Начинаем с центра
    private final double STEP = 0.005;    // Микро-шаг для плавного движения

    @Override
    public void runOpMode() {
        // Инициализация (имя "stopper" должно совпадать с конфигом)
        stopperServo = hardwareMap.get(Servo.class, "stopper");
        stopperServo.setPosition(currentPosition);

        telemetry.addLine("Инструкция:");
        telemetry.addLine("Удерживайте D-Pad ВВЕРХ или ВНИЗ");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Если кнопка зажата — плавно прибавляем или убавляем позицию
            if (gamepad1.dpad_up) {
                currentPosition += STEP;
            } else if (gamepad1.dpad_down) {
                currentPosition -= STEP;
            }

            // Защита, чтобы не выйти за пределы (сервы принимают только от 0.0 до 1.0)
            currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));

            // Отправляем позицию на серву
            stopperServo.setPosition(currentPosition);

            // Выводим данные в реальном времени
            telemetry.addData("Движение", "Удерживайте ВВЕРХ / ВНИЗ");
            telemetry.addData("Позиция сервы", "%.3f", currentPosition);
            telemetry.update();

            // Небольшая задержка цикла, чтобы скорость движения была комфортной
            sleep(20);
        }
    }
}