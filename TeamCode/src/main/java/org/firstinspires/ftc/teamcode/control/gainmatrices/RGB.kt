package org.firstinspires.ftc.teamcode.control.gainmatrices

import org.firstinspires.ftc.teamcode.opmodes.MainAuton

data class RGB @JvmOverloads constructor(
    @JvmField var red: Double = 0.0,
    @JvmField var green: Double = 0.0,
    @JvmField var blue: Double = 0.0,
) {
    fun between(min:RGB, max:RGB): Boolean {

        val redInRange = (min.red <= red) && (red <= max.red)
        val greenInRange = (min.green <= green) && (green <= max.green)
        val blueInRange = (min.blue <= blue) && (blue <= max.blue)

        return redInRange && greenInRange && blueInRange
    }

    fun toTelemetry(title: String) {
        MainAuton.mTelemetry.addLine("$title:")
        MainAuton.mTelemetry.addData("Red", red)
        MainAuton.mTelemetry.addData("Green", green)
        MainAuton.mTelemetry.addData("Blue", blue)
    }
}
