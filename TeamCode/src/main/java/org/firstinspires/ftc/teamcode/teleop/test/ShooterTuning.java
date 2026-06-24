package org.firstinspires.ftc.teamcode.teleop.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.DecodeOpMode;

@TeleOp(name = "Tuning: Shooter", group = "Test")
public class ShooterTuning extends DecodeOpMode {

    // Тестовая скорость, которую мы будем гонять туда-сюда
    private double testTargetRpm = 0;

    // Шаг прибавления скорости на один клик геймпада
    private final double step = 100;

    @Override
    public void onInit() {
        telemetry.addLine("System: Shooter Tuning Ready");
        telemetry.addLine("Controls: D-Pad UP/DOWN to change Target RPM");
        telemetry.addLine("Controls: X (Square) to STOP");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.shooter.turnOn(); // Даем добро на работу моторов
    }

    @Override
    public void onUpdate() {
        // Управление целевой скоростью
        if (base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            testTargetRpm += step;
        } else if (base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            testTargetRpm -= step;
            if (testTargetRpm < 0) testTargetRpm = 0;
        }

        // Экстренная остановка
        if (base.wasJustPressed(GamepadKeys.Button.X)) {
            testTargetRpm = 0;
        }

        // Передаем команду в Шутер (LUT пока НЕ используем)
        robot.shooter.setTargetVelocity(testTargetRpm);

        // ==========================================
        // ВАЖНО: Эти данные нужны для графиков в браузере!
        // ==========================================
        telemetry.addData("1. Target RPM", robot.shooter.getTarget());
        telemetry.addData("2. Current RPM", robot.shooter.getVelocity());
        telemetry.addData("3. Error (RPM)", robot.shooter.getTarget() - robot.shooter.getVelocity());
        telemetry.update();
    }
}