package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.GlobalState;

@TeleOp(name = "⚙️ Настройка Альянса", group = "Setup")
public class AllianceSelector extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addLine("=== ВЫБОР АЛЬЯНСА ===");
        telemetry.addLine("Нажми X (Квадрат) для СИНЕГО");
        telemetry.addLine("Нажми B (Круг) для КРАСНОГО");
        telemetry.update();

        // Цикл работает, пока мы находимся в режиме INIT (до нажатия Play или Stop)
        while (opModeInInit()) {

            if (gamepad1.x || gamepad2.x) {
                GlobalState.isBlueAlliance = true;
            } else if (gamepad1.b || gamepad2.b) {
                GlobalState.isBlueAlliance = false;
            }

            telemetry.addLine("=== ВЫБОР АЛЬЯНСА ===");
            telemetry.addLine("Нажми X (Квадрат) для СИНЕГО");
            telemetry.addLine("Нажми B (Круг) для КРАСНОГО");
            telemetry.addLine("\n------------------------");

            if (GlobalState.isBlueAlliance) {
                telemetry.addData("🔥 ТЕКУЩИЙ АЛЬЯНС", "🟦 СИНИЙ 🟦");
            } else {
                telemetry.addData("🔥 ТЕКУЩИЙ АЛЬЯНС", "🟥 КРАСНЫЙ 🟥");
            }

            telemetry.addLine("------------------------");
            telemetry.addLine("Выбрал? Просто нажми STOP (Квадрат на телефоне).");
            telemetry.update();
        }
    }
}