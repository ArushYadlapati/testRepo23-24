// Please note that this code uses the Logitech F310 as its controller.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp // Without this, this file will not show in the TeleOp section of the REV Driver Hub.
// Note that REV Driver Hub and REV Driver Station are synonymous.
public class TeleOpMode extends LinearOpMode {
    double accelerationFactor = 0.15; // Sets the default movement speed to 15% (0.15).

    // Defines 4 Mecanum Wheel Motors, and then the Viper Slide Motor.
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    // Creates IMU that is set to imu.
    IMU imu;

    String last_button = ""; // Variable that stores the last gamepad1 press/call, which is displayed on REV control hub using telemetry.addData();

    boolean isFieldCentric = true; // Sets the default to field-centric mode when CombinedMecanumTeleOp is initialized on the REV Driver Hub.

    @Override
    public void runOpMode() throws InterruptedException {
        // Declares motors using ID's that match the configuration on the REV Control Hub.
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // Front Left Motor.
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); // Back Left Motor.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // Front Right Motor.
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); // Back Right Motor.

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resets encoder value of viper slide motor to 0.
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the zero power behavior to BRAKE for all motors.
        motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);


        // Sets Viper Slide motor tolerance to 100 (since it uses encoders).
        // This is a fail-safe in case the viper slide is not able to move to its exact encoder value that it needs to go to.
        //motorRightViperSlide.setTargetPositionTolerance(100);
        //motorLeftViperSlide.setTargetPositionTolerance(100);

        // Reverse the right side motors since we are using mecanum wheels.
        // Reverse left motors if you are using NeveRests.
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu"); // Retrieves the IMU from the hardware map.

        // Adjusts the orientation parameters to match the robot (note that IMU is set to imu).
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters); // Without this, the REV Hub's orientation is assumed to be logo up / USB forward.

        // Adds telemetry to the Driver Station.
        telemetry.addData("Status", "Initialized"); // Adds Initialized Status.
        telemetry.addData("Mode", "Field-Centric"); // Since the default mode is Field-Centric, sets Field-Centric to be the mode that is added to REV Driver Hub.
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY).

        imu.resetYaw(); // Resets imu at the start of code.

        // Run until the end of the match (driver presses STOP).
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Toggle control mode on left joystick button press.
            if (gamepad1.left_stick_button) {
                isFieldCentric = !isFieldCentric; // Toggle the mode.
                last_button = "left stick button"; // Sets last button to "left stick button".
                sleep(200); // Small delay to avoid multiple toggles.

                if (isFieldCentric) { // Activates when the mode is Field Centric.
                    telemetry.addData("Mode", "Field-Centric"); // Report the mode change to Field-Centric on Driver Hub.
                } else { // Activates when the mode is Field Centric.
                    telemetry.addData("Mode", "Robot-Centric"); // Report the mode change to Robot-Centric on Driver Hub.
                }

                telemetry.update(); // Adds the mode telemetry to REV Driver Hub.
            }

            // Get raw values from the gamepad.
            double forward = -gamepad1.left_stick_y; // Negative because the gamepad's y-axis is inverted.
            double sideways = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing.
            double rotation = gamepad1.right_stick_x;

            // Use the LT value as an acceleration factor for all mecanum wheel movement.
            // LT value is between 0.15 (not pressed) and 1 (fully pressed).
            double lt = gamepad1.left_trigger;
            double ltSpeed = accelerationFactor + (1 - accelerationFactor) * lt;

            // Reset the yaw angle to 0 degrees when the "Back" button is pressed. Is used for Field-Centric mode, but can be activated during Robot-Centric Mode for Field-Centric mode.
            if (gamepad1.back) {
                imu.resetYaw();
                last_button = "back"; // Sets last button to "back".
            }

            // Resets encoder value of viper slide motor to 0 when the right joystick button is pressed. 
            // Can be used in both Field-Centric and Robot-Centric mode.
            if (gamepad1.right_stick_button) {
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                last_button = "right stick button"; // Sets last button to "right stick button".
            }

            // If the robot is in Field-Centric Mode, the robot will NOT have a head (meaning that the robot's controls WILL NOT change based off the direction it is facing).
            // What direction is forward can be done be resetting the yaw angle to 0 degrees (through pressing gamepad.back).
            if (isFieldCentric) {

                // Calculate motor powers using mecanum drive kinematics.
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the robot's rotation.
                double rotX = sideways * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
                double rotY = sideways * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

                // Denominator is the largest motor power (absolute value) or 1.
                // This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1].
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
                double frontLeftPower = (rotY + rotX + rotation) / denominator * ltSpeed;
                double backLeftPower = (rotY - rotX + rotation) / denominator * ltSpeed;
                double frontRightPower = (rotY - rotX - rotation) / denominator * ltSpeed;
                double backRightPower = (rotY + rotX - rotation) / denominator * ltSpeed;

                // Set motor powers.
                motorFrontLeft.setPower(-frontLeftPower);
                motorBackLeft.setPower(-backLeftPower);
                motorFrontRight.setPower(-frontRightPower);
                motorBackRight.setPower(-backRightPower);

                // Display motorViperSlide encoder position.
                telemetry.addData("Mode:", "Field-Centric"); // Displays current mode (Field-Centric).
                telemetry.addData("Front Left Power", frontLeftPower); // Displays power of the front left mecanum wheel.
                telemetry.addData("Back Left Power", backLeftPower); // Displays power of the back left mecanum wheel.
                telemetry.addData("Front Right Power", frontRightPower); // Displays power of the front right mecanum wheel.
                telemetry.addData("Back Right Power", backRightPower); // Displays power of the back right mecanum wheel.
            }

            // If the robot is in Robot-Centric Mode, the robot will WILL have a head (meaning that the robot's controls WILL change based off the direction it is facing).
            // You can still reset the yaw angle to 0 by using the back button in Robot-Centric mode.
            else {
                // Calculate motor powers using mecanum drive kinematics.
                // No denominator is needed here in Robot-Centric Mode.
                double frontLeftPower = (forward + sideways + rotation) * ltSpeed;
                double frontRightPower = (forward - sideways - rotation) * ltSpeed;
                double backLeftPower = (forward - sideways + rotation) * ltSpeed;
                double backRightPower = (forward + sideways - rotation) * ltSpeed;

                // Set motor powers.
                motorFrontLeft.setPower(-frontLeftPower);
                motorBackLeft.setPower(-backLeftPower);
                motorFrontRight.setPower(-frontRightPower);
                motorBackRight.setPower(-backRightPower);

                telemetry.addData("Mode:", "Robot-Centric"); // Displays current mode (Robot-Centric).
                telemetry.addData("Front Left Power", frontLeftPower); // Displays power of the front left mecanum wheel.
                telemetry.addData("Back Left Power", backLeftPower); // Displays power of the back left mecanum wheel.
                telemetry.addData("Front Right Power", frontRightPower); // Displays power of the front right mecanum wheel.
                telemetry.addData("Back Right Power", backRightPower); // Displays power of the back right mecanum wheel.
            }
            telemetry.addData("Speed (Left Trigger)", ltSpeed); // Displays speed of robot mecanum wheel movement using the left trigger (between 0.15 and 1).
            telemetry.addData("motorFrontRight Position", motorFrontRight.getCurrentPosition());  // Displays motorViperSlide encoder position.
            telemetry.addData("motorFrontLeft Position", motorFrontLeft.getCurrentPosition());  // Displays motorViperSlide encoder position.
            telemetry.addData("motorBackRight Position", motorBackRight.getCurrentPosition());  // Displays motorViperSlide encoder position.
            telemetry.addData("motorBackLeft Position", motorBackLeft.getCurrentPosition());  // Displays motorViperSlide encoder position.
            telemetry.addData("Last button pressed", last_button); // Displays the last gamepad 1 press/call (excluding joystick movement).
            telemetry.update(); // Adds telemetry to REV Driver Hub.
        }
    }
}