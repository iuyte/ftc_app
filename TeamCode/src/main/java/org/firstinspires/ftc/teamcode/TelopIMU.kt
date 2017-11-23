/*
Copyright 2017

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU

import java.util.Locale
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp
class TelopIMU : OpMode() {
    /* Declare OpMode members. */
    private var color: ColorSensor? = null
    private val drive = arrayOfNulls<DcMotor>(4)
    private var button: DigitalChannel? = null
    private var rbutton: DigitalChannel? = null
    private var rev_color: DistanceSensor? = null
    private var sonic: DistanceSensor? = null
    private var imu: BNO055IMU? = null
    private var angles = Orientation()
    private var gravity = Acceleration()
    private var velocity = Velocity()
    private var position = Position()

    override fun init() {
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu!!.initialize(parameters)

        color = hardwareMap.get(ColorSensor::class.java, "color")
        drive[1] = hardwareMap.get(DcMotor::class.java, "front right drive")
        drive[0] = hardwareMap.get(DcMotor::class.java, "front left drive")
        drive[2] = hardwareMap.get(DcMotor::class.java, "back left drive")
        drive[3] = hardwareMap.get(DcMotor::class.java, "back right drive")
        button = hardwareMap.get(DigitalChannel::class.java, "button")
        rbutton = hardwareMap.get(DigitalChannel::class.java, "rbutton")
        rev_color = hardwareMap.get(DistanceSensor::class.java, "rev color")
        sonic = hardwareMap.get(DistanceSensor::class.java, "sonic")

        drive.forEach { motor ->
            motor!!.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        telemetry.addData("Status", "Initialized")
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    override fun init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    override fun start() {
        imu!!.startAccelerationIntegration(position, velocity, 1000)
        telemetry.addAction {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu!!.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            gravity = imu!!.gravity
            velocity = imu!!.velocity
            position = imu!!.position
        }
        telemetry.addLine()
                .addData("heading") { formatAngle(angles.angleUnit, angles.firstAngle.toDouble()) }
                .addData("roll") { formatAngle(angles.angleUnit, angles.secondAngle.toDouble()) }
                .addData("pitch") { formatAngle(angles.angleUnit, angles.thirdAngle.toDouble()) }

        telemetry.addLine()
                .addData("grvty") { gravity.toString() }
                .addData("mag") {
                    String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel * gravity.xAccel
                                    + gravity.yAccel * gravity.yAccel
                                    + gravity.zAccel * gravity.zAccel))
                }
                .addData("vel") { velocity.toString() }
                .addData("pos") { position.toString() }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    override fun loop() {
        val x = gamepad1.left_stick_x
        val y = -gamepad1.left_stick_y
        val θ = gamepad1.right_stick_x
        drive[0]!!.power = (x + y - θ).toDouble()
        drive[1]!!.power = (x - y - θ).toDouble()
        drive[2]!!.power = (-x + y - θ).toDouble()
        drive[3]!!.power = (-x - y - θ).toDouble()
        telemetry.update()
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    override fun stop() {
    }

    private fun formatAngle(angleUnit: AngleUnit, angle: Double): String {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle))
    }

    private fun formatDegrees(degrees: Double): String {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees))
    }
}