package com.imunav.stream

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Typeface
import android.view.View
import java.util.Locale
import kotlin.math.max
import kotlin.math.min

/**
 * The walker's path as the laptop computes and draws it (LivePath): start at the origin, first 3 m pointing up,
 * the dot green while the foot is flat, orange while it swings, grey while calibrating. Same colours as
 * Fusion Player. The view fits the whole path, never narrower than 3 m each side of its centre.
 */
class PathView(context: Context) : View(context) {

    private val dp = context.resources.displayMetrics.density
    private val bg = Paint().apply { color = Color.rgb(250, 251, 252) }
    private val frame = Paint().apply { color = Color.rgb(205, 210, 216); style = Paint.Style.STROKE; strokeWidth = dp }
    private val grid = Paint().apply { color = Color.rgb(228, 231, 235); strokeWidth = dp }
    private val axis = Paint().apply { color = Color.rgb(190, 196, 203); strokeWidth = dp }
    private val line = Paint().apply {
        color = Color.parseColor("#2f6fb0"); style = Paint.Style.STROKE; strokeWidth = 2.5f * dp
        strokeJoin = Paint.Join.ROUND; strokeCap = Paint.Cap.ROUND; isAntiAlias = true
    }
    private val dot = Paint().apply { isAntiAlias = true }
    private val ring = Paint().apply { color = Color.WHITE; style = Paint.Style.STROKE; strokeWidth = 2 * dp; isAntiAlias = true }
    private val big = Paint().apply { textSize = 20 * dp; typeface = Typeface.DEFAULT_BOLD; isAntiAlias = true }
    private val text = Paint().apply { textSize = 15 * dp; color = Color.rgb(31, 35, 40); typeface = Typeface.MONOSPACE; isAntiAlias = true }
    private val note = Paint().apply { textSize = 13 * dp; color = Color.rgb(90, 102, 114); isAntiAlias = true }
    private val path = Path()

    /** set by the screen: whether the service is streaming to a laptop at all */
    var streaming = false

    override fun onDraw(c: Canvas) {
        val w = width.toFloat()
        val h = height.toFloat()
        c.drawRect(0f, 0f, w, h, bg)
        c.drawRect(0f, 0f, w, h, frame)
        val pad = 12 * dp
        val s = LivePath.snapshot()
        if (!streaming || s == null) {
            val msg = if (streaming) "Waiting for the path from the laptop (Fusion Player, Live from the phone)."
                      else "The live path appears here while streaming to Fusion Player on the laptop."
            drawWrapped(c, msg, pad, pad + note.textSize, w - 2 * pad, note)
            return
        }

        // fit: the points, the current position and the start, in a square around their centre
        var lox = min(0f, s.x); var hix = max(0f, s.x); var loy = min(0f, s.y); var hiy = max(0f, s.y)
        val stride = max(1, s.n / 4000)                    // draw at most ~4000 segments
        for (i in 0 until s.n) if (s.have[i]) {
            lox = min(lox, s.xs[i]); hix = max(hix, s.xs[i]); loy = min(loy, s.ys[i]); hiy = max(hiy, s.ys[i])
        }
        val half = max(3f, 0.55f * max(hix - lox, hiy - loy) + 1f)
        val cx = (lox + hix) / 2; val cy = (loy + hiy) / 2
        val top = 64 * dp                                  // room for the text above the plot
        val ph = h - top - pad
        val scale = min(w - 2 * pad, ph) / (2 * half)      // px per metre
        val ox = w / 2; val oy = top + ph / 2
        fun px(x: Float) = ox + (x - cx) * scale
        fun py(y: Float) = oy - (y - cy) * scale

        // grid: a round spacing with at most ~12 lines across
        val step = floatArrayOf(0.5f, 1f, 2f, 5f, 10f, 20f, 50f, 100f, 200f, 500f).firstOrNull { 2 * half / it <= 12 } ?: 1000f
        var g = Math.floor(((cx - w / 2 / scale) / step).toDouble()).toFloat() * step
        while (g <= cx + w / 2 / scale) { c.drawLine(px(g), top, px(g), h, if (g == 0f) axis else grid); g += step }
        g = Math.floor(((cy - (h - oy) / scale) / step).toDouble()).toFloat() * step
        val gTop = cy + (oy - top) / scale
        while (g <= gTop) { c.drawLine(0f, py(g), w, py(g), if (g == 0f) axis else grid); g += step }

        // the path; a point whose packet was lost breaks the line until a later packet fills it in
        path.reset()
        var pen = false
        var last = -1
        var i = 0
        while (i < s.n) {
            if (s.have[i]) {
                if (pen) path.lineTo(px(s.xs[i]), py(s.ys[i])) else path.moveTo(px(s.xs[i]), py(s.ys[i]))
                pen = true; last = i
            } else pen = false
            i += if (i + stride < s.n) stride else max(1, s.n - 1 - i)
        }
        if (last == s.n - 1 && pen) path.lineTo(px(s.x), py(s.y))
        c.drawPath(path, line)
        dot.color = Color.rgb(31, 35, 40)
        c.drawCircle(px(0f), py(0f), 5 * dp, dot); c.drawCircle(px(0f), py(0f), 5 * dp, ring)
        val tracking = s.state == LivePath.TRACKING
        dot.color = Color.parseColor(if (!tracking) "#9aa3ad" else if (s.footFlat) "#2e9e4f" else "#e8a33d")
        c.drawCircle(px(s.x), py(s.y), 8 * dp, dot); c.drawCircle(px(s.x), py(s.y), 8 * dp, ring)

        // status above the plot
        val y1 = pad + big.textSize * 0.8f
        val y2 = y1 + text.textSize * 1.4f
        when {
            s.ageMs > 2000 -> {
                big.color = Color.rgb(179, 38, 58)
                c.drawText("NO PATH FROM LAPTOP (%.0f s)".format(Locale.US, s.ageMs / 1000.0), pad, y1, big)
            }
            tracking -> { big.color = Color.rgb(27, 122, 58); c.drawText("TRACKING", pad, y1, big) }
            else -> { big.color = Color.rgb(178, 106, 0); c.drawText("CALIBRATING", pad, y1, big) }
        }
        val line2 = if (tracking || s.ageMs > 2000)
            "%d step%s   %.1f m   %.2f m/s".format(Locale.US, s.steps, if (s.steps == 1L) "" else "s", s.distance, s.speed)
        else if (s.motion >= 0) "Stand still: shoe motion %.3f rad/s".format(Locale.US, s.motion)
        else "Stand still with the phone on the shoe"
        c.drawText(line2, pad, y2, text)
        val label = "grid ${if (step < 1) "0.5" else step.toInt().toString()} m"
        c.drawText(label, w - pad - note.measureText(label), y2, note)
    }

    private fun drawWrapped(c: Canvas, msg: String, x: Float, y0: Float, width: Float, p: Paint) {
        var y = y0
        var rest = msg
        while (rest.isNotEmpty()) {
            var n = p.breakText(rest, true, width, null)
            if (n < rest.length) rest.lastIndexOf(' ', n).let { if (it > 0) n = it + 1 }
            c.drawText(rest.substring(0, n).trim(), x, y, p)
            rest = rest.substring(n)
            y += p.textSize * 1.4f
        }
    }
}
