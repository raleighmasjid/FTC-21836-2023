package org.firstinspires.ftc.teamcode.control.gainmatrices

data class HSV @JvmOverloads constructor(
    @JvmField var hue: Float = 0.0F,
    @JvmField var saturation: Float = 0.0F,
    @JvmField var value: Float = 0.0F,
) {
    fun inRange(min:HSV, max:HSV): Boolean {

        val hueRange = min.hue..max.hue
        val saturationRange = min.saturation..max.saturation
        val valueRange = min.value..max.value

        return (hue in hueRange) and (saturation in saturationRange) and (value in valueRange)
    }
}
