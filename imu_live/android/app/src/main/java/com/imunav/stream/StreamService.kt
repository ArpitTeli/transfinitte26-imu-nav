package com.imunav.stream

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.net.Uri
import android.net.wifi.WifiManager
import android.os.Build
import android.os.Environment
import android.os.Handler
import android.os.HandlerThread
import android.os.IBinder
import android.os.PowerManager
import android.os.SystemClock
import android.provider.MediaStore
import java.io.BufferedWriter
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStream
import java.io.OutputStreamWriter
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketTimeoutException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Foreground service that records accelerometer and gyroscope events on the phone and, when a laptop address
 * is given, also streams them over UDP.
 *
 * The on-phone recording is the lossless copy: Wi-Fi drops packets (seen as 4-23 % of a walk missing in early
 * laptop recordings), the file on the phone does not. It goes to Download/IMU Stream/ (Android 10+), or to the
 * app's own folder on older phones. A Wi-Fi lock keeps the radio out of power save while streaming.
 *
 * Runs with the screen off (partial wake lock). All sensor callbacks, packing, writing and sending happen on
 * one background thread, so no locking is needed.
 *
 * Wire format (little-endian), must match imunav/live/protocol.py:
 *   "IMU1" | uint32 seq | uint16 count | count x [uint8 type, int64 t_ns, float32 x, y, z]
 *   type 1 = accelerometer (m/s^2, gravity included), 2 = gyroscope (rad/s, uncalibrated)
 * The laptop sends the live path back to the port the samples come from; a listening thread hands those packets
 * to LivePath for the screen to draw.
 * Recording format (read by imunav/live/recording.py): '#' header lines, then  type,t_ns,x,y,z
 */
class StreamService : Service(), SensorEventListener {

    companion object {
        const val EXTRA_HOST = "host"
        const val EXTRA_PORT = "port"
        const val EXTRA_PERIOD_US = "period_us"
        private const val CHANNEL = "imu_stream"
        private const val MAX_RECORDS = 60
        private const val HEADER = 10
        private const val RECORD = 21
        private const val FLUSH_MS = 20L
        private const val REC_DIR = "IMU Stream"

        // Read by MainActivity for the status line.
        @Volatile var running = false
        @Volatile var streaming = false
        @Volatile var accHz = 0.0
        @Volatile var gyroHz = 0.0
        @Volatile var packets = 0L
        @Volatile var gyroKind = ""
        @Volatile var error: String? = null
        @Volatile var recordingName = ""
        @Volatile var recordedEvents = 0L
        @Volatile var accelRangeG = 0.0
        @Volatile var pathPackets = 0L
    }

    private lateinit var sensors: SensorManager
    private var thread: HandlerThread? = null
    private var handler: Handler? = null
    private var socket: DatagramSocket? = null
    private var listener: Thread? = null
    private var target: InetAddress? = null
    private var port = 5555
    private var wakeLock: PowerManager.WakeLock? = null
    private var wifiLock: WifiManager.WifiLock? = null

    private var rec: BufferedWriter? = null
    private var recUri: Uri? = null
    private val line = StringBuilder(64)

    private val buf = ByteBuffer.allocate(HEADER + MAX_RECORDS * RECORD).order(ByteOrder.LITTLE_ENDIAN)
    private var count = 0
    private var seq = 0
    private var accN = 0
    private var gyroN = 0
    private var rateT0 = 0L

    private val flusher = object : Runnable {
        override fun run() {
            flush()
            updateRates()
            handler?.postDelayed(this, FLUSH_MS)
        }
    }

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (running) return START_NOT_STICKY
        val host = intent?.getStringExtra(EXTRA_HOST)?.trim() ?: ""     // empty = record on the phone only
        port = intent?.getIntExtra(EXTRA_PORT, 5555) ?: 5555
        val periodUs = intent?.getIntExtra(EXTRA_PERIOD_US, 0) ?: 0
        startInForeground(if (host.isEmpty()) "recording on the phone" else "to $host:$port")

        wakeLock = (getSystemService(Context.POWER_SERVICE) as PowerManager)
            .newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "imustream:stream").apply { acquire() }
        if (host.isNotEmpty()) acquireWifiLock()
        sensors = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        error = null
        packets = 0
        recordedEvents = 0
        pathPackets = 0
        LivePath.clear()
        streaming = host.isNotEmpty()
        running = true

        val t = HandlerThread("imu-stream", android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY).apply { start() }
        thread = t
        val h = Handler(t.looper)
        handler = h
        h.post {
            val acc = sensors.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
            val gyro = sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
                ?: sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
            if (acc == null || gyro == null) {
                error = "this phone has no accelerometer or gyroscope"
                stopSelf()
                return@post
            }
            gyroKind = if (gyro.type == Sensor.TYPE_GYROSCOPE_UNCALIBRATED) "uncalibrated" else "calibrated"
            accelRangeG = acc.maximumRange / SensorManager.GRAVITY_EARTH.toDouble()
            if (streaming) {
                try {
                    target = InetAddress.getByName(host)
                    socket = DatagramSocket().also { listen(it) }
                } catch (e: Exception) {
                    error = "network: ${e.message} (still recording on the phone)"
                    streaming = false
                }
            }
            try {
                openRecording(acc, gyro, periodUs)
            } catch (e: Exception) {
                error = "cannot save on the phone: ${e.message}"
                if (!streaming) { stopSelf(); return@post }    // nothing to do without a file or a laptop
            }
            buf.clear()
            buf.position(HEADER)
            rateT0 = SystemClock.elapsedRealtime()
            sensors.registerListener(this, acc, periodUs, h)
            sensors.registerListener(this, gyro, periodUs, h)
            h.postDelayed(flusher, FLUSH_MS)
        }
        return START_NOT_STICKY
    }

    /** The live path the laptop sends back, to the port this socket sends from. Ends when the socket closes. */
    private fun listen(s: DatagramSocket) {
        s.soTimeout = 500
        listener = Thread({
            val pkt = DatagramPacket(ByteArray(2048), 2048)
            while (!s.isClosed) {
                try {
                    pkt.length = pkt.data.size       // receive() shrinks it to the last packet's size
                    s.receive(pkt)
                    if (LivePath.update(pkt.data, pkt.length)) pathPackets++
                } catch (_: SocketTimeoutException) {
                } catch (_: Exception) {
                    if (s.isClosed) break
                }
            }
        }, "path-listener").apply { isDaemon = true; start() }
    }

    private fun acquireWifiLock() {
        val wm = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        val mode = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) WifiManager.WIFI_MODE_FULL_LOW_LATENCY
                   else @Suppress("DEPRECATION") WifiManager.WIFI_MODE_FULL_HIGH_PERF
        wifiLock = wm.createWifiLock(mode, "imustream:wifi").apply { setReferenceCounted(false); acquire() }
    }

    private fun openRecording(acc: Sensor, gyro: Sensor, periodUs: Int) {
        val name = "imu_" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()) + ".csv"
        val out: OutputStream
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            val values = ContentValues().apply {
                put(MediaStore.MediaColumns.DISPLAY_NAME, name)
                put(MediaStore.MediaColumns.MIME_TYPE, "text/csv")
                put(MediaStore.MediaColumns.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS + "/" + REC_DIR)
                put(MediaStore.MediaColumns.IS_PENDING, 1)
            }
            val uri = contentResolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                ?: throw IllegalStateException("MediaStore refused the file")
            recUri = uri
            out = contentResolver.openOutputStream(uri) ?: throw IllegalStateException("cannot open $name")
            recordingName = "Download/$REC_DIR/$name"
        } else {
            val dir = File(getExternalFilesDir(null), "recordings").apply { mkdirs() }
            val f = File(dir, name)
            out = FileOutputStream(f)
            recordingName = f.absolutePath
        }
        val w = BufferedWriter(OutputStreamWriter(out, Charsets.US_ASCII), 1 shl 16)
        fun clean(s: String) = s.replace(';', ',').replace('\n', ' ')
        w.write("# IMU Stream recording v1; device=${clean(Build.MANUFACTURER + " " + Build.MODEL)}; android=${Build.VERSION.SDK_INT}" +
                "; accel=${clean(acc.name)}; accel_range=${acc.maximumRange}; accel_resolution=${acc.resolution}" +
                "; gyro=${clean(gyro.name)}; gyro_kind=$gyroKind; gyro_range=${gyro.maximumRange}; gyro_resolution=${gyro.resolution}" +
                "; period_us=$periodUs\n")
        w.write("type,t_ns,x,y,z\n")
        rec = w
    }

    private fun closeRecording() {
        try { rec?.close() } catch (_: Exception) {}
        rec = null
        recUri?.let { uri ->
            try {
                contentResolver.update(uri, ContentValues().apply { put(MediaStore.MediaColumns.IS_PENDING, 0) }, null, null)
            } catch (_: Exception) {}
        }
        recUri = null
    }

    override fun onSensorChanged(e: SensorEvent) {
        val type = when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> { accN++; 1 }
            else -> { gyroN++; 2 }  // uncalibrated gyro: values[0..2] are the raw rates
        }
        rec?.let { w ->
            // Float.toString is locale-independent ('.' decimal point), unlike String.format
            line.setLength(0)
            line.append(type).append(',').append(e.timestamp).append(',')
                .append(e.values[0]).append(',').append(e.values[1]).append(',').append(e.values[2]).append('\n')
            try {
                w.append(line)
                recordedEvents++
            } catch (ex: Exception) {
                error = "saving stopped: ${ex.message}"
                rec = null
            }
        }
        if (!streaming) return
        buf.put(type.toByte())
        buf.putLong(e.timestamp)
        buf.putFloat(e.values[0])
        buf.putFloat(e.values[1])
        buf.putFloat(e.values[2])
        count++
        if (count == MAX_RECORDS) flush()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    private fun flush() {
        if (count == 0) return
        val s = socket
        if (s == null) {                 // never let the packet buffer overflow when there is no socket
            count = 0; buf.clear(); buf.position(HEADER); return
        }
        val end = buf.position()
        buf.position(0)
        buf.put(byteArrayOf('I'.code.toByte(), 'M'.code.toByte(), 'U'.code.toByte(), '1'.code.toByte()))
        buf.putInt(seq++)
        buf.putShort(count.toShort())
        try {
            s.send(DatagramPacket(buf.array(), end, target, port))
            packets++
        } catch (e: Exception) {
            error = "send: ${e.message}"  // keep going; Wi-Fi may come back (the phone copy is complete anyway)
        }
        count = 0
        buf.clear()
        buf.position(HEADER)
    }

    private fun updateRates() {
        val now = SystemClock.elapsedRealtime()
        val dt = (now - rateT0) / 1000.0
        if (dt >= 1.0) {
            accHz = accN / dt
            gyroHz = gyroN / dt
            accN = 0
            gyroN = 0
            rateT0 = now
        }
    }

    private fun startInForeground(text: String) {
        val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        nm.createNotificationChannel(NotificationChannel(CHANNEL, "IMU streaming", NotificationManager.IMPORTANCE_LOW))
        val open = PendingIntent.getActivity(
            this, 0, Intent(this, MainActivity::class.java), PendingIntent.FLAG_IMMUTABLE
        )
        val n = Notification.Builder(this, CHANNEL)
            .setContentTitle("IMU Stream")
            .setContentText(text)
            .setSmallIcon(android.R.drawable.stat_sys_upload)
            .setContentIntent(open)
            .setOngoing(true)
            .build()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            startForeground(1, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC)
        } else {
            startForeground(1, n)
        }
    }

    override fun onDestroy() {
        running = false
        streaming = false
        val h = handler
        val t = thread
        if (h != null && t != null) {
            h.post {
                if (::sensors.isInitialized) sensors.unregisterListener(this)
                h.removeCallbacks(flusher)
                flush()
                socket?.close()
                closeRecording()
                t.quitSafely()
            }
        }
        wakeLock?.let { if (it.isHeld) it.release() }
        wifiLock?.let { if (it.isHeld) it.release() }
        accHz = 0.0
        gyroHz = 0.0
        super.onDestroy()
    }
}
