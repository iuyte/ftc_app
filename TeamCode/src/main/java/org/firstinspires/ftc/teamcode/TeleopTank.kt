package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by ethan on 7/29/17.
 */

@TeleOp(name = "robot: Teleop Tank", group = "robot")

class TeleopTank : OpMode() {

    /* Declare OpMode members. */
    private var robot = Hardware(this, Hardware.DriveMode.Tank) // use the class created to define a robot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    override fun init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap)

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Robot", "Initialized") //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    override fun init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    override fun start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    override fun loop() {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        val left: Float = -gamepad1.left_stick_y
        val right: Float = -gamepad1.right_stick_y

        robot.setDrive(left, right, left, right)

        // Send telemetry message to signify robot running;
        telemetry.addData("Left", "%.2f", left)
        telemetry.addData("Right", "%.2f", right)
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    override fun stop() {}

}
