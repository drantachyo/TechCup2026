package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Tuner (Single Press)", group = "Tuning")
public class HoodTun extends LinearOpMode {

    private Servo hoodServo;

    private double currentPosition = 0.5;
    private final double STEP = 0.01; // Увеличили шаг, так как теперь двигаем по кликам

    // Сохраненные позиции
    private double lowerLimit = -1.0;
    private double upperLimit = -1.0;

    // Переменные для защиты от залипания (хранят состояние кнопок из прошлого цикла)
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevA = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() {
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(currentPosition);

        telemetry.addLine("=== ИНСТРУКЦИЯ ===");
        telemetry.addLine("D-Pad ВВЕРХ/ВНИЗ : Шаг 0.01 (1 клик = 1 шаг)");
        telemetry.addLine("Кнопка A (Крест) : Сохранить как НИЖНЕЕ положение");
        telemetry.addLine("Кнопка Y (Треуг.): Сохранить как ВЕРХНЕЕ положение");
        telemetry.addLine("Бамперы (LB/RB)  : Прыжок в сохраненные позиции");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Считываем текущее состояние кнопок
            boolean currDpadUp = gamepad1.dpad_up;
            boolean currDpadDown = gamepad1.dpad_down;
            boolean currA = gamepad1.a;
            boolean currY = gamepad1.y;

            // --- 1. УПРАВЛЕНИЕ ПО КЛИКАМ ---
            // Сработает только в момент нажатия (сейчас нажато, а в прошлом цикле - нет)
            if (currDpadUp && !prevDpadUp) {
                currentPosition += STEP;
            } else if (currDpadDown && !prevDpadDown) {
                currentPosition -= STEP;
            }

            // --- 2. СОХРАНЕНИЕ ПОЗИЦИЙ ---
            if (currA && !prevA) {
                lowerLimit = currentPosition;
            }
            if (currY && !prevY) {
                upperLimit = currentPosition;
            }

            // Переключение для теста бамперами оставил обычным (пока держишь - едет туда)
            if (gamepad1.left_bumper && lowerLimit != -1.0) {
                currentPosition = lowerLimit;
            }
            if (gamepad1.right_bumper && upperLimit != -1.0) {
                currentPosition = upperLimit;
            }

            // Защита от выхода за пределы 0.0 и 1.0
            currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));
            hoodServo.setPosition(currentPosition);

            // --- 3. ОБНОВЛЯЕМ ПРЕДЫДУЩИЕ СОСТОЯНИЯ ---
            // Обязательно в конце цикла запоминаем, что кнопки были нажаты
            prevDpadUp = currDpadUp;
            prevDpadDown = currDpadDown;
            prevA = currA;
            prevY = currY;

            // --- 4. ТЕЛЕМЕТРИЯ ---
            telemetry.addData("Текущая позиция", "%.3f", currentPosition);
            telemetry.addLine("-------------------------");
            telemetry.addData("Нижний лимит", lowerLimit == -1.0 ? "НЕ ЗАДАН" : String.format("%.3f", lowerLimit));
            telemetry.addData("Верхний лимит", upperLimit == -1.0 ? "НЕ ЗАДАН" : String.format("%.3f", upperLimit));
            telemetry.update();
        }
    }
}