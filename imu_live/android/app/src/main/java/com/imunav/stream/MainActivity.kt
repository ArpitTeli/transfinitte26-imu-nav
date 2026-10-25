package com.imunav.stream

import android.Manifest
import android.app.Activity
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Typeface
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.text.InputType
import android.view.Gravity
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.RadioButton
import android.widget.RadioGroup
import android.widget.TextView
import java.util.Locale

/** One screen: laptop address, sample rate, Start/Stop, live status, and the live path the laptop sends back. */
class MainActivity : Activity() {

    private lateinit var intro: TextView
    private lateinit var host: EditText
    private lateinit var port: EditText
    private lateinit var rates: RadioGroup
    private lateinit var button: Button
    private lateinit var status: TextView
    private lateinit var pathView: PathView
    private val ui = Handler(Looper.getMainLooper())

    // label -> sensor sampling period in microseconds (0 = SENSOR_DELAY_FASTEST)
    private val rateOptions = listOf("Fastest" to 0, "400 Hz" to 2500, "200 Hz" to 5000)

    private val tick = object : Runnable {
        override fun run() {
            refresh()
            ui.postDelayed(this, 500)
        }
    }

    private val pathTick = object : Runnable {
        override fun run() {
            pathView.streaming = StreamService.streaming
            pathView.invalidate()
            ui.postDelayed(this, 100)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val prefs = getSharedPreferences("imustream", Context.MODE_PRIVATE)
        val pad = (16 * resources.displayMetrics.density).toInt()

        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(pad, pad * 2, pad, pad)
        }
        root.addView(TextView(this).apply {
            text = "IMU Stream"
            textSize = 26f
            setTypeface(typeface, Typeface.BOLD)
        })
        intro = TextView(this).apply {
            text = "Strap the phone to the shoe. Start, then keep the foot still for 2 s. " +
                "Every session is also saved on the phone (Download/IMU Stream), so nothing is lost if Wi-Fi drops. " +
                "Keep this screen open while walking: Wi-Fi stays fastest with the screen on. " +
                "The laptop sends the path back, and it is drawn below."
            setPadding(0, pad / 2, 0, pad)
        }
        root.addView(intro)
        root.addView(TextView(this).apply { text = "Laptop IP (shown in the laptop app; leave empty to record on the phone only)" })
        host = EditText(this).apply {
            setText(prefs.getString("host", "192.168.1.100"))
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            isSingleLine = true
        }
        root.addView(host)
        root.addView(TextView(this).apply { text = "Port" })
        port = EditText(this).apply {
            setText(prefs.getInt("port", 5555).toString())
            inputType = InputType.TYPE_CLASS_NUMBER
            isSingleLine = true
        }
        root.addView(port)
        root.addView(TextView(this).apply { text = "Sample rate"; setPadding(0, pad / 2, 0, 0) })
        rates = RadioGroup(this).apply { orientation = RadioGroup.HORIZONTAL }
        rateOptions.forEachIndexed { i, (label, _) ->
            rates.addView(RadioButton(this).apply { text = label; id = 100 + i })
        }
        rates.check(100 + prefs.getInt("rate", 0))
        root.addView(rates)
        button = Button(this).apply {
            textSize = 20f
            setOnClickListener { toggle() }
        }
        root.addView(button, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT
        ).apply { topMargin = pad })
        status = TextView(this).apply {
            typeface = Typeface.MONOSPACE
            setPadding(0, pad, 0, 0)
            gravity = Gravity.START
        }
        root.addView(status)
        pathView = PathView(this)
        root.addView(pathView, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f)
            .apply { topMargin = pad / 2 })
        setContentView(root)

        if (Build.VERSION.SDK_INT >= 33 &&
            checkSelfPermission(Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED
        ) {
            requestPermissions(arrayOf(Manifest.permission.POST_NOTIFICATIONS), 1)
        }
    }

    override fun onResume() {
        super.onResume()
        ui.post(tick)
        ui.post(pathTick)
    }

    override fun onPause() {
        ui.removeCallbacks(tick)
        ui.removeCallbacks(pathTick)
        super.onPause()
    }

    private fun toggle() {
        if (StreamService.running) {
            stopService(Intent(this, StreamService::class.java))
        } else {
            val h = host.text.toString().trim()                       // empty = record on the phone only
            val p = port.text.toString().toIntOrNull()
            if (p == null || p !in 1..65535) {
                status.text = "Enter a port between 1 and 65535."
                return
            }
            val rateIdx = rates.checkedRadioButtonId - 100
            getSharedPreferences("imustream", Context.MODE_PRIVATE).edit()
                .putString("host", h).putInt("port", p).putInt("rate", rateIdx).apply()
            val i = Intent(this, StreamService::class.java)
                .putExtra(StreamService.EXTRA_HOST, h)
                .putExtra(StreamService.EXTRA_PORT, p)
                .putExtra(StreamService.EXTRA_PERIOD_US, rateOptions[rateIdx].second)
            startForegroundService(i)
        }
        ui.postDelayed({ refresh() }, 300)
    }

    private fun refresh() {
        val on = StreamService.running
        button.text = if (on) "Stop" else "Start"
        intro.visibility = if (on) View.GONE else View.VISIBLE   // room for the path while running
        host.isEnabled = !on
        port.isEnabled = !on
        for (i in 0 until rates.childCount) rates.getChildAt(i).isEnabled = !on
        status.text = buildString {
            append(if (!on) "stopped\n" else if (StreamService.streaming) "STREAMING + RECORDING\n" else "RECORDING ON THE PHONE\n")
            append("accel %.0f Hz   gyro %.0f Hz   mag %.0f Hz\n".format(Locale.US, StreamService.accHz, StreamService.gyroHz, StreamService.magHz))
            if (StreamService.streaming) append("packets sent ${StreamService.packets}   path packets back ${StreamService.pathPackets}\n")
            if (StreamService.recordingName.isNotEmpty()) {
                append("saved: ${StreamService.recordingName}\n")
                append("events saved ${StreamService.recordedEvents}\n")
            }
            if (StreamService.gyroKind.isNotEmpty()) append("gyro: ${StreamService.gyroKind}\n")
            if (StreamService.accelRangeG > 0) append("accel range +-%.0f g\n".format(Locale.US, StreamService.accelRangeG))
            StreamService.error?.let { append("error: $it\n") }
        }
    }
}
