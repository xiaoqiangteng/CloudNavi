/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.examples.java.pointcloud;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.tango.support.TangoPointCloudManager;
import com.google.tango.support.TangoSupport;
import com.google.tango.ux.TangoUx;
import com.google.tango.ux.UxExceptionEvent;
import com.google.tango.ux.UxExceptionEventListener;


import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.display.DisplayManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import org.rajawali3d.scene.ASceneFrameCallback;
import org.rajawali3d.surface.RajawaliSurfaceView;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the {@link Tango}
 * service and propagation of Tango point cloud data to OpenGL and Layout views. OpenGL rendering
 * logic is delegated to the {@link PointCloudRajawaliRenderer} class.
 */
public class PointCloudActivity extends Activity implements View.OnClickListener {
    private static final String TAG = PointCloudActivity.class.getSimpleName();
    private static final String TAG1 = "teng1";
    private static final String UX_EXCEPTION_EVENT_DETECTED = "Exception Detected: ";
    private static final String UX_EXCEPTION_EVENT_RESOLVED = "Exception Resolved: ";
    private static final int SECS_TO_MILLISECS = 1000;
    private static final DecimalFormat FORMAT_THREE_DECIMAL = new DecimalFormat("0.000");
    private static final double UPDATE_INTERVAL_MS = 100.0;

    private Tango mTango;
    private TangoConfig mConfig;
    private TangoUx mTangoUx;

    private TangoPointCloudManager mPointCloudManager;
    private PointCloudRajawaliRenderer mRenderer;
    private RajawaliSurfaceView mSurfaceView;
    private TextView mPointCountTextView;
    private TextView mAverageZTextView;
    private TextView mFileWrittenCount;
    private EditText mFileName;
    private Button mSnapShot;
    private Button mRecord;
    private Button mStart;
    private Button mEnd;
    private Switch mIsSensor;

    private double mPointCloudPreviousTimeStamp;
    private boolean mIsConnected = false;
    private double mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;
    private int mDisplayRotation = 0;
    private boolean isSnapShot;
    private boolean isRecord;

    private String strFileDirPointCloud = "";
    private int mNumberOfFilesWritten;
    private SensorManager mSensorManager;
    private boolean isBegin;
    private double globaltimestamp;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private FileOutputStream fosAcce;
    private FileOutputStream fosGyro;
    private FileOutputStream fosOrit;
    private FileOutputStream fosMagn;
    private FileOutputStream fosPress;
    private FileOutputStream fosGrav;
    private String strFileDirSensor;
    private boolean isSensorBegin;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_point_cloud);

        mPointCountTextView = (TextView) findViewById(R.id.point_count_textview);
        mAverageZTextView = (TextView) findViewById(R.id.average_z_textview);
        mFileWrittenCount = (TextView) findViewById(R.id.file_written_count);
        mSurfaceView = (RajawaliSurfaceView) findViewById(R.id.gl_surface_view);
        mFileName = (EditText) findViewById(R.id.file_name);
        mSnapShot = (Button) findViewById(R.id.take_snap_button);
        mSnapShot.setEnabled(false);
        mRecord = (Button) findViewById(R.id.record_switch);
        mRecord.setEnabled(false);
        mStart = (Button) findViewById(R.id.start);
        mEnd = (Button) findViewById(R.id.end);
        mSnapShot.setOnClickListener(this);
        mRecord.setOnClickListener(this);
        mStart.setOnClickListener(this);
        mEnd.setOnClickListener(this);
        mEnd.setEnabled(false);
        isSensorBegin = false;
        mIsSensor = (Switch) findViewById(R.id.isSensor);
        mIsSensor.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                isSensorBegin = isChecked;
            }
        });

        mPointCloudManager = new TangoPointCloudManager();
        mTangoUx = setupTangoUxAndLayout();
        mRenderer = new PointCloudRajawaliRenderer(this);
        setupRenderer();
        DisplayManager displayManager = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        if (displayManager != null) {
            displayManager.registerDisplayListener(new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {
                }

                @Override
                public void onDisplayChanged(int displayId) {
                    synchronized (this) {
                        setDisplayRotation();
                    }
                }

                @Override
                public void onDisplayRemoved(int displayId) {
                }
            }, null);
        }
        isSnapShot = false;
        isRecord = false;
        mNumberOfFilesWritten = 0;
        isBegin = false;
        init();
        strFileDirPointCloud = "";
        strFileDirSensor = "";
        mIsSensor.setChecked(false);
    }

    @Override
    public void onClick(View view) {
        switch (view.getId()) {
            case R.id.start:
                //Point clouds
                creatFolder();
                mSnapShot.setEnabled(true);
                mRecord.setEnabled(true);
                mEnd.setEnabled(true);
                mStart.setEnabled(false);
                //inertial sensor
                fosAcce = createOutputFile(strFileDirSensor, "ACCE.txt");
                fosGyro = createOutputFile(strFileDirSensor, "Gyro.txt");
                fosOrit = createOutputFile(strFileDirSensor, "Orit.txt");
                fosMagn = createOutputFile(strFileDirSensor, "Magn.txt");
                fosPress = createOutputFile(strFileDirSensor, "Press.txt");
                fosGrav = createOutputFile(strFileDirSensor, "Grav.txt");
                globaltimestamp = System.nanoTime();
                break;
            case R.id.take_snap_button:
                isSnapShot = true;
                break;
            case R.id.record_switch:
                isRecord = true;
                isBegin = true;
                mRecord.setEnabled(false);
                mSnapShot.setEnabled(false);
                break;
            case R.id.end:
                try {
                    fosAcce.write("\n\n".getBytes());
                    fosAcce.flush();
                    fosAcce.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
                try {
                    fosOrit.write("\n\n".getBytes());
                    fosOrit.flush();
                    fosOrit.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
                try{
                    fosGyro.write("\n\n".getBytes());
                    fosGyro.flush();
                    fosGyro.close();
                }catch (IOException e) {
                    e.printStackTrace();
                }
                try{
                    fosMagn.write("\n\n".getBytes());
                    fosMagn.flush();
                    fosMagn.close();
                }catch (IOException e) {
                    e.printStackTrace();
                }
                try{
                    fosPress.write("\n\n".getBytes());
                    fosPress.flush();
                    fosPress.close();
                }catch (IOException e) {
                    e.printStackTrace();
                }
                try{
                    fosGrav.write("\n\n".getBytes());
                    fosGrav.flush();
                    fosGrav.close();
                }catch (IOException e) {
                    e.printStackTrace();
                }
                isBegin = false;
                isRecord = false;
                mStart.setEnabled(true);
                mSnapShot.setEnabled(false);
                mRecord.setEnabled(false);
                mEnd.setEnabled(false);
                break;
            default:
                Log.w(TAG, "Unrecognized button click.");
                break;
        }

    }

    private void creatFolder(){
        Calendar rightNow = Calendar.getInstance();
        int hour = rightNow.get(Calendar.HOUR_OF_DAY);
        int minute = rightNow.get(Calendar.MINUTE);
        int sec = rightNow.get(Calendar.SECOND);
        int milliSec = rightNow.get(Calendar.MILLISECOND);
        String mNowTimeString = "" + (int)(1000000 * hour + 10000 * minute + 100 * sec +
                (float)milliSec / 10.0);
        String fileName = null;
        if("".equals(mFileName.getText().toString().trim()))
        {
            fileName = "";
        } else {
            fileName = "_" + mFileName.getText().toString();
        }
        strFileDirPointCloud = Environment.getExternalStorageDirectory()+ "/a_PointClouds/" + mNowTimeString + fileName + "/pointclouds/";
        Toast.makeText(this, strFileDirPointCloud,Toast.LENGTH_LONG).show();
        strFileDirSensor = Environment.getExternalStorageDirectory()+ "/a_PointClouds/" + mNowTimeString + fileName + "/sensor/";
        Toast.makeText(this, strFileDirSensor,Toast.LENGTH_LONG).show();
    }

    private void init(){
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        //注册加速度传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_NORMAL)) {
            Log.i(TAG, "加速度传感器可用！");
        } else {
            Log.i(TAG, "加速度传感器不可用！");
        }

        // 注册地磁场传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_NORMAL)) {
            Log.i(TAG, "地磁传感器可用！");
        } else {
            Log.i(TAG, "地磁传感器不可用！");
        }

        // 注册陀螺仪传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_NORMAL)) {
            Log.i(TAG, "陀螺仪传感器可用！");
        } else {
            Log.i(TAG, "陀螺仪传感器不可用！");
        }

        // 注册方向传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION),
                SensorManager.SENSOR_DELAY_NORMAL)) {
            Log.i(TAG, "方向传感器可用！");
        } else {
            Log.i(TAG, "方向传感器不可用！");
        }

        // 注册重力传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY),
                SensorManager.SENSOR_DELAY_NORMAL)) {
            Log.i(TAG, "重力传感器可用！");
        } else {
            Log.i(TAG, "重力传感器不可用！");
        }

        // 注册线性加速度传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                SensorManager.SENSOR_DELAY_NORMAL)) {
            Log.i(TAG, "线性加速度传感器可用！");
        } else {
            Log.i(TAG, "线性加速度传感器不可用！");
        }

        // 注册气压传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE),
                SensorManager.SENSOR_DELAY_NORMAL)) {
            Log.i(TAG, "气压传感器可用！");
        } else {
            Log.i(TAG, "气压传感器不可用！");
        }
    }

    @Override
    protected void onStart() {
        super.onStart();

        mTangoUx.start();
        bindTangoService();
    }

    @Override
    protected void onStop() {
        super.onStop();
        // Synchronize against disconnecting while the service is being used in the OpenGL
        // thread or in the UI thread.
        // NOTE: DO NOT lock against this same object in the Tango callback thread.
        // Tango.disconnect will block here until all Tango callback calls are finished.
        // If you lock against this object in a Tango callback thread it will cause a deadlock.
        synchronized (this) {
            try {
                mTangoUx.stop();
                mTango.disconnect();
                mIsConnected = false;
            } catch (TangoErrorException e) {
                Log.e(TAG, getString(R.string.exception_tango_error), e);
            }
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // 注销传感器监听
        mSensorManager.unregisterListener((SensorEventListener) this);
        try {
            fosAcce.close();
            fosGrav.close();
            fosGyro.close();
            fosMagn.close();
            fosPress.close();
            fosPress.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private SensorEventListener sensorListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            if (isSensorBegin){
                if (isBegin){
//                    if(sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
//                        StringBuilder sb = new StringBuilder();
//                        sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
//                        sb.append(sensorEvent.values[0]+"\t");
//                        sb.append(sensorEvent.values[1]+"\t");
//                        sb.append(sensorEvent.values[2]+"\t\n");
//                        try {
//                            fosAcce.write(sb.toString().getBytes());
//                            fosAcce.flush();
//                        } catch (IOException e) {
//                            e.printStackTrace();
//                        }
//                    }
//                    if(sensorEvent.sensor.getType() == Sensor.TYPE_GYROSCOPE){
//                        StringBuilder sb = new StringBuilder();
//                        sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
//                        sb.append(sensorEvent.values[0]+"\t");
//                        sb.append(sensorEvent.values[1]+"\t");
//                        sb.append(sensorEvent.values[2]+"\t\n");
//                        try {
//                            fosGyro.write(sb.toString().getBytes());
//                            fosGyro.flush();
//                        } catch (IOException e) {
//                            e.printStackTrace();
//                        }
//                    }

//                    if(sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
//                        StringBuilder sb = new StringBuilder();
//                        sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
//                        sb.append(sensorEvent.values[0]+"\t");
//                        sb.append(sensorEvent.values[1]+"\t");
//                        sb.append(sensorEvent.values[2]+"\t\n");
//                        try {
//                            fosMagn.write(sb.toString().getBytes());
//                            fosMagn.flush();
//                        } catch (IOException e) {
//                            e.printStackTrace();
//                        }
//                    }
//                    if(sensorEvent.sensor.getType() == Sensor.TYPE_GRAVITY){
//                        StringBuilder sb = new StringBuilder();
//                        sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
//                        sb.append(sensorEvent.values[0]+"\t");
//                        sb.append(sensorEvent.values[1]+"\t");
//                        sb.append(sensorEvent.values[2]+"\t\n");
//                        try {
//                            fosGrav.write(sb.toString().getBytes());
//                            fosGrav.flush();
//                        } catch (IOException e) {
//                            e.printStackTrace();
//                        }
//                    }
                    if(sensorEvent.sensor.getType() == Sensor.TYPE_ORIENTATION){
                        StringBuilder sb = new StringBuilder();
                        sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
                        sb.append(sensorEvent.values[0]+"\t");
                        sb.append(sensorEvent.values[1]+"\t");
                        sb.append(sensorEvent.values[2]+"\t\n");
                        try {
                            fosOrit.write(sb.toString().getBytes());
                            fosOrit.flush();
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }
//                    if(sensorEvent.sensor.getType() == Sensor.TYPE_PRESSURE){
//                        StringBuilder sb = new StringBuilder();
//                        sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
//                        sb.append(sensorEvent.values[0]+"\t\n");
//                        try {
//                            fosPress.write(sb.toString().getBytes());
//                            fosPress.flush();
//                        } catch (IOException e) {
//                            e.printStackTrace();
//                        }
//                    }
                }
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    /**
     * Initialize Tango Service as a normal Android Service.
     */
    private void bindTangoService() {
        // Initialize Tango Service as a normal Android Service. Since we call mTango.disconnect()
        // in onPause, this will unbind Tango Service, so every time onResume gets called we
        // should create a new Tango object.
        mTango = new Tango(PointCloudActivity.this, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready; this Runnable
            // will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only when there are no
            // UI thread changes involved.
            @Override
            public void run() {
                // Synchronize against disconnecting while the service is being used in the OpenGL
                // thread or in the UI thread.
                synchronized (PointCloudActivity.this) {
                    try {
                        mConfig = setupTangoConfig(mTango);
                        mTango.connect(mConfig);
                        startupTango();
                        TangoSupport.initialize(mTango);
                        mIsConnected = true;
                        setDisplayRotation();
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, getString(R.string.exception_out_of_date), e);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, getString(R.string.exception_tango_error), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_error);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, getString(R.string.exception_tango_invalid), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_invalid);
                    }
                }
            }
        });
    }

    /**
     * Sets up the Tango configuration object. Make sure mTango object is initialized before
     * making this call.
     */
    private TangoConfig setupTangoConfig(Tango tango) {
        // Use the default configuration plus add depth sensing.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);
        return config;
    }

    /**
     * Set up the callback listeners for the Tango Service and obtain other parameters required
     * after Tango connection.
     * Listen to updates from the Point Cloud and Tango Events and Pose.
     */
    private void startupTango() {
        final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();

        framePairs.add(new TangoCoordinateFramePair(TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            @Override
            public void onPoseAvailable(TangoPoseData pose) {
                // Passing in the pose data to UX library produce exceptions.
                if (mTangoUx != null) {
                    mTangoUx.updatePoseStatus(pose.statusCode);
                }
            }

            @Override
            public void onPointCloudAvailable(final TangoPointCloudData pointCloud) {
                if (mTangoUx != null) {
                    mTangoUx.updatePointCloud(pointCloud);
                }
                mPointCloudManager.updatePointCloud(pointCloud);
                final double currentTimeStamp = pointCloud.timestamp;
                final double pointCloudFrameDelta =
                        (currentTimeStamp - mPointCloudPreviousTimeStamp) * SECS_TO_MILLISECS;
                mPointCloudPreviousTimeStamp = currentTimeStamp;
                final double averageDepth = getAveragedDepth(pointCloud.points,
                        pointCloud.numPoints);
                mPointCloudTimeToNextUpdate -= pointCloudFrameDelta;
                if (mPointCloudTimeToNextUpdate < 0.0) {
                    mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;
                    final String pointCountString = Integer.toString(pointCloud.numPoints);
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mPointCountTextView.setText(pointCountString);
                            mAverageZTextView.setText(FORMAT_THREE_DECIMAL.format(averageDepth));
                            mFileWrittenCount.setText(String.format("%04d", mNumberOfFilesWritten));
                        }
                    });
                }
                //Write point clouds
                if (isSnapShot){
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            mNumberOfFilesWritten ++;
                            writePointCloudToFile(pointCloud);
                        }
                    }).start();
                    isSnapShot = false;
                }
                if (isRecord){
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            mNumberOfFilesWritten ++;
                            writePointCloudToFile(pointCloud);
                        }
                    }).start();
                }
            }

            @Override
            public void onTangoEvent(TangoEvent event) {
                if (mTangoUx != null) {
                    mTangoUx.updateTangoEvent(event);
                }
            }
        });
    }

    // This function writes the XYZ points to .vtk files in binary
    private void writePointCloudToFile(TangoPointCloudData pointCloud) {
        FileOutputStream fosPointClouds;
        Calendar rightNow = Calendar.getInstance();
        int hour = rightNow.get(Calendar.HOUR_OF_DAY);
        int minute = rightNow.get(Calendar.MINUTE);
        int sec = rightNow.get(Calendar.SECOND);
        int milliSec = rightNow.get(Calendar.MILLISECOND);
        String mNowTimeString = "" + (int)(1000000 * hour + 10000 * minute + 100 * sec +
                (float)milliSec / 10.0);
        String pointCloudFileName = mNowTimeString + "_" + String.format("%04d", mNumberOfFilesWritten) + ".txt";
        fosPointClouds = createOutputFile(strFileDirPointCloud, pointCloudFileName);
        int numFloatWrite = 4 * pointCloud.numPoints;
        for (int i = 0; i < numFloatWrite; i += 4){
            StringBuilder sb = new StringBuilder();
            sb.append(pointCloud.points.get(i) + "\t");
            sb.append(pointCloud.points.get(i+1) + "\t");
            sb.append(pointCloud.points.get(i+2) + "\n");
            try {
                fosPointClouds.write(sb.toString().getBytes());
                fosPointClouds.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        try {
            fosPointClouds.write("\n\n".getBytes());
            fosPointClouds.flush();
            fosPointClouds.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private FileOutputStream createOutputFile(String path, String fileName) {
        File dir = new File(path);
        if(!dir.exists() && !dir.isDirectory()){
            dir.mkdirs();
        }
        FileOutputStream fos = null;
        File file = new File(path, fileName);
        if(!file.exists()) {
            try {
                file.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        try {
            fos = new FileOutputStream(file, true);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        return fos;
    }

    /**
     * Sets Rajawali surface view and its renderer. This is ideally called only once in onCreate.
     */
    public void setupRenderer() {
        mSurfaceView.setEGLContextClientVersion(2);
        mRenderer.getCurrentScene().registerFrameCallback(new ASceneFrameCallback() {
            @Override
            public void onPreFrame(long sceneTime, double deltaTime) {
                // NOTE: This will be executed on each cycle before rendering; called from the
                // OpenGL rendering thread.

                // Prevent concurrent access from a service disconnect through the onPause event.
                synchronized (PointCloudActivity.this) {
                    // Don't execute any Tango API actions if we're not connected to the service.
                    if (!mIsConnected) {
                        return;
                    }

                    // Update point cloud data.
                    TangoPointCloudData pointCloud = mPointCloudManager.getLatestPointCloud();
                    if (pointCloud != null) {
                        // Calculate the depth camera pose at the last point cloud update.
                        TangoSupport.MatrixTransformData transform =
                                TangoSupport.getMatrixTransformAtTime(pointCloud.timestamp,
                                        TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                        TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                        TangoSupport.ENGINE_OPENGL,
                                        TangoSupport.ENGINE_TANGO,
                                        TangoSupport.ROTATION_IGNORED);
                        if (transform.statusCode == TangoPoseData.POSE_VALID) {
                            mRenderer.updatePointCloud(pointCloud, transform.matrix);
                        }
                    }

                    // Update current camera pose.
                    try {
                        // Calculate the device pose. This transform is used to display
                        // frustum in third and top down view, and used to render camera pose in
                        // first person view.
                        TangoPoseData lastFramePose = TangoSupport.getPoseAtTime(0,
                                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                TangoPoseData.COORDINATE_FRAME_DEVICE,
                                TangoSupport.ENGINE_OPENGL,
                                TangoSupport.ENGINE_OPENGL,
                                mDisplayRotation);
                        if (lastFramePose.statusCode == TangoPoseData.POSE_VALID) {
                            mRenderer.updateCameraPose(lastFramePose);
                        }
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "Could not get valid transform");
                    }
                }
            }

            @Override
            public boolean callPreFrame() {
                return true;
            }

            @Override
            public void onPreDraw(long sceneTime, double deltaTime) {

            }

            @Override
            public void onPostFrame(long sceneTime, double deltaTime) {

            }
        });
        mSurfaceView.setSurfaceRenderer(mRenderer);
    }

    /**
     * Sets up TangoUX and sets its listener.
     */
    private TangoUx setupTangoUxAndLayout() {
        TangoUx tangoUx = new TangoUx(this);
        tangoUx.setUxExceptionEventListener(mUxExceptionListener);
        return tangoUx;
    }

    /*
    * Set a UxExceptionEventListener to be notified of any UX exceptions.
    * In this example we are just logging all the exceptions to logcat, but in a real app,
    * developers should use these exceptions to contextually notify the user and help direct the
    * user in using the device in a way Tango Service expects it.
    * <p>
    * A UxExceptionEvent can have two statuses: DETECTED and RESOLVED.
    * An event is considered DETECTED when the exception conditions are observed, and RESOLVED when
    * the root causes have been addressed.
    * Both statuses will trigger a separate event.
    */
    private UxExceptionEventListener mUxExceptionListener = new UxExceptionEventListener() {
        @Override
        public void onUxExceptionEvent(UxExceptionEvent uxExceptionEvent) {
            String status = uxExceptionEvent.getStatus() == UxExceptionEvent.STATUS_DETECTED ?
                    UX_EXCEPTION_EVENT_DETECTED : UX_EXCEPTION_EVENT_RESOLVED;

            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_LYING_ON_SURFACE) {
                Log.i(TAG, status + "Device lying on surface");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_DEPTH_POINTS) {
                Log.i(TAG, status + "Too few depth points");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_FEATURES) {
                Log.i(TAG, status + "Too few features");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOTION_TRACK_INVALID) {
                Log.i(TAG, status + "Invalid poses in MotionTracking");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOVING_TOO_FAST) {
                Log.i(TAG, status + "Moving too fast");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_OVER_EXPOSED) {
                Log.i(TAG, status + "Fisheye Camera Over Exposed");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_UNDER_EXPOSED) {
                Log.i(TAG, status + "Fisheye Camera Under Exposed");
            }
        }
    };

    /**
     * First Person button onClick callback.
     */
    public void onFirstPersonClicked(View v) {
        mRenderer.setFirstPersonView();
    }

    /**
     * Third Person button onClick callback.
     */
    public void onThirdPersonClicked(View v) {
        mRenderer.setThirdPersonView();
    }

    /**
     * Top-down button onClick callback.
     */
    public void onTopDownClicked(View v) {
        mRenderer.setTopDownView();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        mRenderer.onTouchEvent(event);
        return true;
    }

    /**
     * Calculates the average depth from a point cloud buffer.
     *
     * @param pointCloudBuffer
     * @param numPoints
     * @return Average depth.
     */
    private float getAveragedDepth(FloatBuffer pointCloudBuffer, int numPoints) {
        float totalZ = 0;
        float averageZ = 0;
        if (numPoints != 0) {
            int numFloats = 4 * numPoints;
            for (int i = 2; i < numFloats; i = i + 4) {
                totalZ = totalZ + pointCloudBuffer.get(i);
            }
            averageZ = totalZ / numPoints;
        }
        return averageZ;
    }

    /**
     * Query the display's rotation.
     */
    private void setDisplayRotation() {
        Display display = getWindowManager().getDefaultDisplay();
        mDisplayRotation = display.getRotation();
    }

    /**
     * Display toast on UI thread.
     *
     * @param resId The resource id of the string resource to use. Can be formatted text.
     */
    private void showsToastAndFinishOnUiThread(final int resId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(PointCloudActivity.this,
                        getString(resId), Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }

}
