package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Created by ethan on 11/25/17.
 */

@TeleOp(name = "Motor Test", group = "Robot")
class BatteryTest : OpMode() {
    private var motor : DcMotor? = null
    private var elasped : Double? = null
    private val period = ElapsedTime()
    private var startVoltage : Double = 0.0

    override fun init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        motor = hardwareMap.dcMotor.get("motor")
        motor!!.power = 0.0
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    override fun init_loop() {}

    // Code to run ONCE when the driver hits PLAY
    override fun start() {
        motor!!.power = 1.0
        startVoltage = getBatteryVoltage()
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    override fun loop() {
        val v = getBatteryVoltage()
        if (v > 11) {
            motor!!.power = 1.0
            telemetry.addData("time", period.seconds().toLong())
        } else {
            motor!!.power = 0.0
            if (elasped == null) {
                elasped = v
            }
            telemetry.addData("died at", elasped)
        }

        telemetry.addData("starting voltage", startVoltage)
                 .addData("power", v)
                 .addData("voltage drop", startVoltage - v)
                 .addData(
                         "drop rate",
                         "%f mV/ms",
                         (startVoltage - v) * 1000 / period.milliseconds())
        telemetry.update()
    }

    // Code to run ONCE after the driver hits STOP
    override fun stop() {}

    private fun getBatteryVoltage(): Double {
        var result = java.lang.Double.POSITIVE_INFINITY
        hardwareMap.voltageSensor.forEach { sensor ->
            val voltage = sensor.voltage
            if (voltage > 0) {
                result = Math.min(result, voltage)
            }
        }
        return result
    }
}