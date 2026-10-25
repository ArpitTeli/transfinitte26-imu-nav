package com.imunav.stream

import android.os.SystemClock
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * The live path the laptop sends back (Fusion Player, PTH1 packets), so the phone draws what the laptop draws.
 *
 * Wire format (little-endian), must match imunav/live/protocol.py:
 *   "PTH1" | uint16 gen | uint8 state | uint8 flags | uint32 steps | float32 distance, speed, motion, x, y |
 *   uint32 points in the path | uint32 first point here | uint16 count | count x [float32 x, float32 y]
 * A packet carries any run of the path's points, so a lost one is filled in by a later packet. A new gen means
 * the laptop replaced the path (recalibrated, moved the origin, or turned it to point up): the old points go.
 *
 * Written by the service's listening thread, read by the screen; every access holds the object's lock.
 */
object LivePath {
    private const val HEADER = 42
    private const val MAX_POINTS = 2_000_000          // 200 km at 0.1 m per point: anything bigger is not a path
    const val CALIBRATING = 0
    const val TRACKING = 1
    private const val FLAG_STILL = 1

    /** A copy for drawing. have[i] is false for a point still missing (its packet was lost). */
    class Snapshot(
        val state: Int, val footFlat: Boolean, val steps: Long, val distance: Float, val speed: Float,
        val motion: Float, val x: Float, val y: Float, val n: Int, val xs: FloatArray, val ys: FloatArray,
        val have: BooleanArray, val ageMs: Long,
    )

    private var gen = -1
    private var state = CALIBRATING
    private var flags = 0
    private var steps = 0L
    private var distance = 0f
    private var speed = 0f
    private var motion = -1f
    private var cx = 0f
    private var cy = 0f
    private var n = 0
    private var xs = FloatArray(1024)
    private var ys = FloatArray(1024)
    private var have = BooleanArray(1024)
    private var lastMs = 0L

    @Synchronized fun clear() {
        gen = -1; n = 0; lastMs = 0L
        have.fill(false)
    }

    /** Take one packet; false if it is not a valid path packet (it is then ignored). */
    @Synchronized fun update(data: ByteArray, length: Int): Boolean {
        if (length < HEADER) return false
        val b = ByteBuffer.wrap(data, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (b.get() != 'P'.code.toByte() || b.get() != 'T'.code.toByte() ||
            b.get() != 'H'.code.toByte() || b.get() != '1'.code.toByte()) return false
        val g = b.short.toInt() and 0xFFFF
        val st = b.get().toInt() and 0xFF
        val fl = b.get().toInt() and 0xFF
        val stp = b.int.toLong() and 0xFFFFFFFFL
        val dist = b.float
        val spd = b.float
        val mot = b.float
        val x = b.float
        val y = b.float
        val total = b.int
        val start = b.int
        val count = b.short.toInt() and 0xFFFF
        if (total < 0 || total > MAX_POINTS || start < 0 || start.toLong() + count > total ||
            length != HEADER + 8 * count) return false
        if (g != gen) {
            gen = g
            have.fill(false)
        }
        if (total > xs.size) {
            var size = xs.size
            while (size < total) size *= 2
            xs = xs.copyOf(size); ys = ys.copyOf(size); have = have.copyOf(size)
        }
        for (i in start until start + count) {
            xs[i] = b.float
            ys[i] = b.float
            have[i] = true
        }
        n = total; state = st; flags = fl; steps = stp
        distance = dist; speed = spd; motion = mot; cx = x; cy = y
        lastMs = SystemClock.elapsedRealtime()
        return true
    }

    /** null until the first packet arrives */
    @Synchronized fun snapshot(): Snapshot? {
        if (lastMs == 0L) return null
        return Snapshot(
            state, flags and FLAG_STILL != 0, steps, distance, speed, motion, cx, cy, n,
            xs.copyOf(n), ys.copyOf(n), have.copyOf(n), SystemClock.elapsedRealtime() - lastMs,
        )
    }
}
