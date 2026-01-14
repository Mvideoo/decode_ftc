// В начале файла обозначается пакет (папка), в котором он находится
package org.firstinspires.ftc.teamcode;
// Затем идет импорт внешних классов или функций или из них

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Пометка или аннотация над объявлением класса добавляет его
 * в меню DS как TeleOp или Autonomous соответственно
 */
@TeleOp
/*
 * Название класса должно соответствовать названию файла
 * "extends LinearOpMode" делает класс (файл) запускаемой программой
 */
public class test42 extends LinearOpMode {
    DcMotor m1 = null;
    Servo myServo = null; // Объявляем сервопривод

    @Override
    public void runOpMode() {
        m1 = hardwareMap.get(DcMotor.class, "m1");
        myServo = hardwareMap.get(Servo.class, "s1");
        myServo.setPosition(0.5);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && myServo.getPosition() < 1) {
                myServo.setPosition(myServo.getPosition() + 0.05);
                sleep(200);
            }
            if (gamepad1.b && myServo.getPosition() > 0) {
                myServo.setPosition(myServo.getPosition() - 0.05);
                sleep(200);

            }
        }
        if (myServo.getPosition() < 0.5) {
            m1.setPower(myServo.getPosition() * (-2));
        } else if (myServo.getPosition() == 0.5) {
            m1.setPower(myServo.getPosition() * (0));
        } else if (myServo.getPosition() > 0.5) {
            m1.setPower(myServo.getPosition() * (2));
        }
    }
}