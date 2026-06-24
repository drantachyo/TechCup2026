package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Hood;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

public class Robot {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final Hood hood;
    public final Stopper stopper;

    // ПУБЛИЧНЫЙ КОНСТРУКТОР: Создаем новые подсистемы при каждом вызове new Robot()
    public Robot() {
        drive = new Drive();
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        hood = new Hood();
        stopper = new Stopper();
    }

    /**
     * Инициализация железа с передачей стартовой позиции
     */
    public void init(HardwareMap hw, Pose startPose) {
        drive.init(hw);
        drive.setPose(startPose); // Скармливаем координаты в Pedro Pathing

        intake.init(hw);
        shooter.init(hw);
        turret.init(hw);
        hood.init(hw);
        stopper.init(hw);
    }

    public void periodic() {
        drive.periodic();
        intake.periodic();
        shooter.periodic();
        turret.periodic();
    }
}