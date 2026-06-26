package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// Импорт ваших подсистем
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Stopper;

@TeleOp(name = "Master Master Tuner", group = "Tuning")
public class ShooterTun_All extends LinearOpMode {

    // Подсистемы и оборудование
    private Shooter shooter;
    private DcMotor intakeMotor;
    private Servo hoodServo;
    private Servo stopperServo;

    // --- НАСТРОЙКИ ШАГОВ ---
    private final double HOOD_STEP = 0.003; // Плавность капюшона
    private final double RPM_STEP = 50.0;   // Шаг изменения скорости шутера

    // Текущие значения (капюшон управляется локально)
    private double hoodPosition = 0.5;

    // Переменные состояния ТелеОпа
    private boolean isStopperOpen = false;

    // Переменные для защиты от залипания (Edge Detection)
    private boolean prevA = false;
    private boolean prevX = false; // Добавлено для тугла шутера
    private boolean prevRT = false;
    private boolean prevLT = false;

    @Override
    public void runOpMode() {
        // Инициализация оборудования
        shooter = new Shooter();
        shooter.init(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        stopperServo = hardwareMap.get(Servo.class, "stopper");

        // Установка начальных позиций
        hoodServo.setPosition(hoodPosition);
        stopperServo.setPosition(Stopper.CLOSED_POS);

        telemetry.addLine("=== УПРАВЛЕНИЕ MASTER TUNER ===");
        telemetry.addLine("D-Pad ВВЕРХ/ВНИЗ : Интейк (вперед/назад)");
        telemetry.addLine("Триггеры (RT/LT) : Скорость шутера (+/- 50)");
        telemetry.addLine("Бамперы (RB/LB)  : Угол капюшона (удерживать)");
        telemetry.addLine("Кнопка A (Крест) : Стоппер (Открыть/Закрыть)");
        telemetry.addLine("Кнопка X (Квадрат): ШУТЕР (Включить/Выключить тугл)");
        telemetry.addLine("Кнопка B (Круг)  : ЭКСТРЕННО ВЫКЛЮЧИТЬ ШУТЕР");
        telemetry.update();

        waitForStart();

        // По умолчанию выключим шутер при старте, чтобы он не крутился сам
        shooter.turnOff();
        shooter.setTargetVelocity(Shooter.targetVelocity);

        while (opModeIsActive()) {

            // --- 1. ИНТЕЙК (D-Pad) ---
            if (gamepad1.dpad_up) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.dpad_down) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            // --- 2. КАПЮШОН (Бамперы) ---
            if (gamepad1.right_bumper) {
                hoodPosition += HOOD_STEP;
            } else if (gamepad1.left_bumper) {
                hoodPosition -= HOOD_STEP;
            }

            hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));
            hoodServo.setPosition(hoodPosition);

            // --- 3. ШУТЕР (Управление скоростью и Тугл на X) ---
            boolean currRT = gamepad1.right_trigger > 0.5;
            boolean currLT = gamepad1.left_trigger > 0.5;
            boolean currX = gamepad1.x; // Считываем кнопку X

            // Изменение целевой скорости
            if (currRT && !prevRT) {
                Shooter.targetVelocity += RPM_STEP;
                shooter.setTargetVelocity(Shooter.targetVelocity);
            } else if (currLT && !prevLT) {
                Shooter.targetVelocity -= RPM_STEP;
                Shooter.targetVelocity = Math.max(0, Shooter.targetVelocity);
                shooter.setTargetVelocity(Shooter.targetVelocity);
            }

            // ТУГЛ РАБОТЫ ШУТЕРА НА КНОПКУ X
            if (currX && !prevX) {
                shooter.toggle();
                // Если в результате переключения шутер активировался,
                // принудительно обновляем ему целевую скорость
                if (Shooter.isActivated) {
                    shooter.setTargetVelocity(Shooter.targetVelocity);
                }
            }

            // Экстренный стоп на B
            if (gamepad1.b) {
                shooter.turnOff();
            }

            // --- 4. СТОППЕР (Кнопка А - Toggle) ---
            boolean currA = gamepad1.a;
            if (currA && !prevA) {
                isStopperOpen = !isStopperOpen;
                if (isStopperOpen) {
                    stopperServo.setPosition(Stopper.OPEN_POS);
                } else {
                    stopperServo.setPosition(Stopper.CLOSED_POS);
                }
            }

            // Вызов периодических расчетов шутера (PIDF + компенсация)
            shooter.periodic();

            // Обновляем состояния кнопок для защиты от залипания
            prevRT = currRT;
            prevLT = currLT;
            prevA = currA;
            prevX = currX; // Запоминаем состояние X

            // --- 5. ТЕЛЕМЕТРИЯ ---
            telemetry.addData("--- СОСТОЯНИЕ МЕХАНИЗМОВ ---", "");
            telemetry.addData("Шутер Моторы", Shooter.isActivated ? "РАБОТАЮТ" : "ВЫКЛЮЧЕНЫ");
            telemetry.addData("Шутер (Target | Current)", "%.0f | %.0f", Shooter.targetVelocity, shooter.getVelocity());
            telemetry.addData("Капюшон Позиция (ЗАПИСАТЬ)", "%.3f", hoodPosition);
            telemetry.addData("Стоппер Состояние", isStopperOpen ? "ОТКРЫТ (%.2f)" : "ЗАКРЫТ (%.2f)",
                    isStopperOpen ? Stopper.OPEN_POS : Stopper.CLOSED_POS);

            telemetry.addLine();
            telemetry.addData("--- ТЮНИНГ КОЭФФИЦИЕНТОВ ШУТЕРА (LIVE) ---", "");
            telemetry.addData("kS (Статическое трение)", "%.4f", Shooter.kS);
            telemetry.addData("kP (Пропорциональный)", "%.5f", Shooter.kP);
            telemetry.addData("kV1 (Мин. скорость)", "%.6f", Shooter.kV1);
            telemetry.addData("kV2 (Макс. скорость)", "%.6f", Shooter.kV2);
            telemetry.update();
        }
    }
}