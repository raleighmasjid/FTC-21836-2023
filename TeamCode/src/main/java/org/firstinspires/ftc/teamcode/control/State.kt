package org.firstinspires.ftc.teamcode.control

data class State @JvmOverloads constructor(
    @JvmField val x: Double = 0.0,
    @JvmField val v: Double = 0.0,
    @JvmField val a: Double = 0.0,
    @JvmField val j: Double = 0.0,
) {

    operator fun plus(other:State): State {
        return State(
            x + other.x,
            v + other.v,
            a + other.a,
            j + other.j,
        )
    }

    operator fun unaryMinus(): State {
        return State(
            -x,
            -v,
            -a,
            -j,
        )
    }

    operator fun minus(other:State): State {
        return this + -other
    }
}
