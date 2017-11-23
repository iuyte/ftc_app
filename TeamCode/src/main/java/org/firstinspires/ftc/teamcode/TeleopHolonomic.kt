package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

/**"Color",
 * Created by ethan on 7/29/17.
 */

@TeleOp(name = "Teleop Holonomic", group = "Robot")

class TeleopHolonomic : OpMode() {

    // Manages some reusable code
    private var robot = Hardware(this, Hardware.DriveMode.Holonomic, true)

    // Code to run ONCE when the driver hits INIT
    override fun init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    override fun init_loop() {}

    // Code to run ONCE when the driver hits PLAY
    override fun start() {}

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    override fun loop() {
        // Run wheels in holonomic mode
        robot.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x)

        telemetry.addData("Red", robot.colorSensor!!.red())
        telemetry.addData("Green", robot.colorSensor!!.green())
        telemetry.addData("Blue", robot.colorSensor!!.blue())
        telemetry.addData("Color", robot.color())
        telemetry.addData("Touch", robot.button!!.state)
        telemetry.addData("Rev Touch", robot.revButton!!.state)

        // Sleep the thread until the loop time surpasses 25ms
        robot.waitForTick(25)
    }

    // Code to run ONCE after the driver hits STOP
    override fun stop() {}

}
