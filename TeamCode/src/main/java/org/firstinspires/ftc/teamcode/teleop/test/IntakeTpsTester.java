package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@TeleOp(name = "⚙️ Тест TPS Интейка", group = "Test")
public class IntakeTpsTester extends LinearOpMode {

    // Подключаем твой класс интейка
    private Intake intake;

    @Override
    public void runOpMode() {
        intake = new Intake();
        intake.init(hardwareMap);

        telemetry.addLine("Готов к тесту TPS!");
        telemetry.addLine("Нажми Play, затем зажимай A или B на геймпаде 1.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Управление интейком (А - вперед, B - реверс)
            if (gamepad1.a) {
                intake.setPower(1.0);
            } else if (gamepad1.b) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            // Читаем скорость из твоего нового метода
            double currentTps = intake.getVelocity();

            // Выводим всё в Телеметрию
            telemetry.addLine("=== ТЕСТ СКОРОСТИ ИНТЕЙКА ===");
            telemetry.addData("Состояние", gamepad1.a ? "ВКЛ (1.0)" : (gamepad1.b ? "РЕВЕРС (-1.0)" : "ВЫКЛ (0.0)"));

            // 🔥 ВОТ ЭТА ЦИФРА НАМ И НУЖНА
            telemetry.addData("🚀 ТЕКУЩИЙ TPS", currentTps);

            telemetry.addLine("\n--- ИНСТРУКЦИЯ ---");
            telemetry.addLine("1. Зажми 'A' вхолостую. Запиши цифру TPS.");
            telemetry.addLine("2. Засунь 3 элемента (заблокируй вал). Зажми 'A'.");
            telemetry.addLine("3. Запиши упавшую цифру TPS.");
            telemetry.addLine("4. Впиши в Автоном число ровно между ними!");
            telemetry.update();
        }
    }
}