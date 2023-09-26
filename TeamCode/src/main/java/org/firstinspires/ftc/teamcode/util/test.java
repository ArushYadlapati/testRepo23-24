// The Seattle Solver's code for the 2023-2024 FTC Season (CENTERSTAGE).
// Please note that this code uses the Logitech F310 as its controller.
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp // Without this, this file will not show in the TeleOp section of the REV Driver Hub.
// Note that REV Driver Hub and REV Driver Station are synonymous.
public class test extends LinearOpMode {
    double accelerationFactor = 0.15; // Sets the default movement speed to 15% (0.15).

    // Defines 4 Mecanum Wheel Motors.
    DcMotor testMotor;

    int targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declares motors using ID's that match the configuration on the REV Control Hub.
        testMotor = hardwareMap.dcMotor.get("testMotor"); // Back Right Motor.

        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        testMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        waitForStart(); // Wait for the game to start (driver presses PLAY).

        // Run until the end of the match (driver presses STOP).
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                testMotor.setPower(-1);
                testMotor.setTargetPosition(targetPosition -= 1000); // This sets the target position of the right viper slide to newViperSlidePosition.
                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This causes the right viper slide motor to move to the value of newViperSlidePosition.
            }

            if (gamepad1.b) {
                testMotor.setPower(1);
                testMotor.setTargetPosition(targetPosition += 1000); // This sets the target position of the right viper slide to newViperSlidePosition.
                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This causes the right viper slide motor to move to the value of newViperSlidePosition.
            }
        }
    }
}