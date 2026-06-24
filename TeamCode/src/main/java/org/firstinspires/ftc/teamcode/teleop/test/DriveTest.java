package org.firstinspires.ftc.teamcode.teleop.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.DecodeOpMode;

@TeleOp(name = "Test: Drive Only", group = "Test")
public class DriveTest extends DecodeOpMode {

    private boolean isBlueAlliance = true;

    @Override
    public void onInit() {
        telemetry.addLine("System: Drive Test Ready.");
        telemetry.addLine("Controls:");
        telemetry.addLine(" - Sticks: Move & Rotate");
        telemetry.addLine(" - A (Cross): Toggle Field-Centric");
        telemetry.addLine(" - B (Circle): Toggle Alliance (Red/Blue)");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.drive.startTeleop();
    }

    @Override
    public void onUpdate() {
        // 1. Управление движением
        robot.drive.drive(base, isBlueAlliance);

        // 2. Переключение режимов (Field-Centric / Robot-Centric)
        if (base.wasJustPressed(GamepadKeys.Button.A)) {
            robot.drive.toggleFieldCentric();
        }

        // 3. Переключение цвета альянса (меняет ориентацию поля на 180 градусов)
        if (base.wasJustPressed(GamepadKeys.Button.B)) {
            isBlueAlliance = !isBlueAlliance;
        }

        // ==========================================
        // ТЕЛЕМЕТРИЯ (Смотреть в FTC Dashboard)
        // ==========================================
        Pose currentPose = robot.drive.getPose();

        telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
        telemetry.addData("X (Inches)", currentPose.getX());
        telemetry.addData("Y (Inches)", currentPose.getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }
}