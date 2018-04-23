package life.orient;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

/**
 * 方向传感器
 */

public class OrientSensor implements SensorEventListener {
    private static final String TAG = "OrientSensor";
    private SensorManager sensorManager;
    private OrientCallBack orientCallBack;
    private Context context;
    private float[] accelerometerValues = new float[3];
    private float[] magneticValues = new float[3];
    private float[] oriValues = new float[3];
    private float[] gyroValues = new float[3];
    private float[] gravValues = new float[3];
    private float[] lAccValues = new float[3];
//    private SensorValuesCallBack sensorValuesCallBack;

    public OrientSensor(Context context, OrientCallBack orientCallBack) {
        this.context = context;
        this.orientCallBack = orientCallBack;
//        this.sensorValuesCallBack = sensorValuesCallBack;
    }

    public interface OrientCallBack {
        /**
         * 方向回调
         */
        void Orient(int orient);
    }

//    public interface SensorValuesCallBack {
//        /**
//         * 传感器数据回调
//         */
//        void SensorValues(float[] acce, float[] magn, float[] gyro, float[] ori, float[] grav, float[] lacc);
//    }

    /**
     * 注册传感器
     * @return 是否支持这些传感器功能
     */
    public Boolean registerOrient() {
        Boolean isAvailable = true;
        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        // 注册方向传感器
        if (sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION),
                SensorManager.SENSOR_DELAY_GAME)) {
            Log.i(TAG, "方向传感器可用！");
        } else {
            Log.i(TAG, "方向传感器不可用！");
            isAvailable = false;
        }
        return isAvailable;
    }

    /**
     * 注销方向监听器
     */
    public void unregisterOrient() {
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ORIENTATION){
            oriValues = event.values.clone();
        }
        orientCallBack.Orient((int) oriValues[0]);
//        sensorValuesCallBack.SensorValues(accelerometerValues, magneticValues, gyroValues, oriValues, gravValues, lAccValues);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}
