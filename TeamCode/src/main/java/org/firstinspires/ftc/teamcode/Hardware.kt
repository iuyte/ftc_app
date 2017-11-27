package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables

/**
 * Created by ethan on 7/29/17.
 */

/* Makes moving with the robot easier */
class Hardware(private var hwMap : HardwareMap?,
               private var mode : OpMode,
               private var driveMode : DriveMode,
               private var useSensors : Boolean = false,
               private var useVuforia : Boolean = false,
               private var useMotors : Boolean = true) {
    // Color sensor
    var colorSensor: ColorSensor? = null
    var button: DigitalChannel? = null
    var revButton: DigitalChannel? = null

    // Vuforia members
    var vuforia: VuforiaLocalizer? = null
    private var cameraMonitorViewId : Int? = null
    private var vuforiaParams : VuforiaLocalizer.Parameters? = null
    private val vuforiaKey =
            "AQ+eJzP/////AAAAGQ6Eyl3tQ0fqvcl2vinKWkR8t2wTESOSuHo32BhQvWxRBBxt+4C2BtrXTh" +
                    "Ygcyd9fzaZ1tTjW8viQzNJAVYyL50GIG+hKJb6mHRq/iYJ53ve+oETR1t5ZBaVmq2l" +
                    "qljNaxyRJURYSLp52UoNi2RpvW0xvVIZ4KBoClIUI4KRkiozACNg0GsxDIDnAiBx5i" +
                    "iPfug5PklSyPnKHkhKf+mtRhoY8HcittFKh4lQkI5LtJOm6lH/K6CCa7RzVUkRbI9m" +
                    "bxVybXiOLtpN/Yr+FDS0bM19czvflS+URyrO2J5APOipQ1XZ1wieP/wp75NFXkV0V3" +
                    "hWcVAkWHQr9aJhZtkYUGjrb94QP7OOF7h6Ftl+ls2j"

    // VuMark specific
    var relicTrackables : VuforiaTrackables? = null
    var relicTemplate : VuforiaTrackable? = null

    enum class COLORS {
        RED,
        BLUE,
        NONE,
    }

    /* The motors on the drive */
    var motors: MutableMap<String, Array<DcMotor>> = mutableMapOf()

    /* private OpMode members. */
    private val period = ElapsedTime()

    /* The different drive modes */
    enum class DriveMode {
        Tank,
        Holonomic,
        Mecanum,
    }

    /* Initialize standard Hardware interfaces */
    fun init(awMap : HardwareMap?, motorMode : DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        hwMap = awMap!!
        if (useSensors) {
            // Initialize sensors
            colorSensor = hwMap!!.get(ColorSensor::class.java, "rev color")
            button = hwMap!!.get(DigitalChannel::class.java, "button")
            revButton = hwMap!!.get(DigitalChannel::class.java, "rbutton")
        }

        if (useMotors) {
            // Define and Initialize Motors
            motors["drive"] = arrayOf(
                    hwMap!!.dcMotor["front left drive"]!!,
                    hwMap!!.dcMotor["front right drive"]!!,
                    hwMap!!.dcMotor["back left drive"]!!,
                    hwMap!!.dcMotor["back right drive"]!!
            )

            motors["arm"] = arrayOf(
                    hwMap!!.dcMotor["left lift motor"]!!,
                    hwMap!!.dcMotor["right lift motor"]!!,
                    hwMap!!.dcMotor["claw control motor"]!!,
                    hwMap!!.dcMotor["claw motor"]!!
            )

            // Set all motors to zero power and to run without encoders
            motors["drive"]!!.forEach {
                it.power = 0.0
                it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            }

            motors["arm"]!!.forEach {
                it.power = 0.0
                it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            }


            Thread.sleep(50)
            motors["drive"]!!.forEach {
                it.mode = motorMode
            }

            motors["drive"]!!.forEach {
                it.mode = motorMode
            }

            motors["drive"]!![0].direction = DcMotorSimple.Direction.REVERSE
            motors["drive"]!![1].direction = DcMotorSimple.Direction.FORWARD
            motors["drive"]!![2].direction = DcMotorSimple.Direction.REVERSE
            motors["drive"]!![3].direction = DcMotorSimple.Direction.FORWARD
            motors["arm"]!![0].direction = DcMotorSimple.Direction.FORWARD
            motors["arm"]!![1].direction = DcMotorSimple.Direction.REVERSE
        }

        if (useVuforia) {
            // Vuforia initialization, setup
            cameraMonitorViewId = hwMap!!.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hwMap!!.appContext.packageName)
            vuforiaParams = VuforiaLocalizer.Parameters(cameraMonitorViewId!!)
            vuforiaParams!!.vuforiaLicenseKey = vuforiaKey
            vuforiaParams!!.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
            this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams)

            relicTrackables = this.vuforia!!.loadTrackablesFromAsset("RelicVuMark")
            relicTemplate = relicTrackables!![0]
            relicTemplate!!.name = "relicVuMarkTemplate"
        }

        mode.telemetry.addData("Robot", "Initialized!")
    }

    fun setDrive(upLeft: Float, upright: Float, downLeft: Float, downright: Float) {
        motors["drive"]!![0].power = upLeft.toDouble()
        motors["drive"]!![1].power = upright.toDouble()
        motors["drive"]!![2].power = downLeft.toDouble()
        motors["drive"]!![3].power = downright.toDouble()

        // Send telemetry message to signify robot running;
        motors["drive"]!!.forEach {
            mode.telemetry.addData(it.deviceName, "%d", it.currentPosition)
        }
    }

    private fun holonomic(x : Float, y : Float, θ : Float) {
        val fl = -x + y - θ
        val fr =  x + y + θ
        val bl =  x + y - θ
        val br = -x + y + θ
        setDrive(fl, fr, bl, br)
    }

    private fun mecanum(x : Float, y : Float, θ : Float) {
        val fl = x + y + θ
        val fr = x + y + θ
        val bl = x + y + θ
        val br = x + y + θ
        setDrive(fl, fr, bl, br)
    }

    fun drive(x : Float, y : Float, θ : Float) {
        when (driveMode) {
            DriveMode.Holonomic -> holonomic(x, y, θ)
            DriveMode.Mecanum -> mecanum(x, y, θ)
            else -> mode.telemetry.addData("Error", "Called Hardware::Drive, current driveMode not found")
        }
    }

    fun color(c : ColorSensor? = colorSensor) : COLORS {
        val rgb = arrayOf(
                c!!.red(),
                c.blue(),
                c.green())
        if (rgb[0] > rgb[2] + 25) {
            return COLORS.RED
        } else if (rgb[2] > rgb[0] + 15) {
            return COLORS.BLUE
        }
        return COLORS.NONE
    }

    fun vuMark() : RelicRecoveryVuMark = RelicRecoveryVuMark.from(relicTemplate)

    /***

     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.

     * @param periodMs Length of wait cycle in mSec.
     */
    fun waitForTick(periodMs: Long) {
        val remaining = periodMs - period.milliseconds().toLong()

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining)
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset()
    }
}
