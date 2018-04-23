package life;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

import life.orient.OrientSensor;
import life.step.StepSensorAcceleration;
import life.step.StepSensorBase;
import life.util.SensorUtil;

public class MainActivity extends AppCompatActivity implements StepSensorBase.StepCallBack,
        OrientSensor.OrientCallBack, View.OnClickListener {//Runnable,
    private String TAG = "mainActivity";
    private TextView mStepText;
    private TextView mOrientText;
    private EditText etStationID;
    private EditText etExitID;
    private EditText etPathID;
    private Button btnStart;
    private Button btnEnd;
    private StepView mStepView;
    private StepSensorBase mStepSensor; // 计步传感器
    private OrientSensor mOrientSensor; // 方向传感器
    private int mStepLen = 10; // 步长
    //Write file
    private String strFileDir = "";
    private FileOutputStream fosAcce;
    private FileOutputStream fosGyro;
    private FileOutputStream fosOrit;
    private FileOutputStream fosMagn;
    private FileOutputStream fosPress;
    private FileOutputStream fosLineAcc;
    private FileOutputStream fosGrav;
    private FileOutputStream fosPath;
    //path
    private float pathX = 500;
    private float pathY = 800;
    private int mOrient = 0;
    private boolean isPathWrite = false;
    //sensor data
    private boolean isBegin = false;
    private SensorManager mSensorManager;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private double globaltimestamp;

    @Override
    public void Step(int stepNum) {
        //  计步回调
        mStepText.setText("步数:" + stepNum);
        if (isBegin){
            mStepView.autoAddPoint(mStepLen);
            pathX += (float) (mStepLen * Math.sin(Math.toRadians(mOrient)));
            pathY += -(float) (mStepLen * Math.cos(Math.toRadians(mOrient)));
            isPathWrite = true;
        } else {
            pathX = 500;
            pathY = 800;
        }
    }

    @Override
    public void Orient(int orient) {
        // 方向回调
        mOrientText.setText("方向:" + orient);
        mStepView.autoDrawArrow(orient);
        mOrient = orient;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        SensorUtil.getInstance().printAllSensor(this); // 打印所有可用传感器
        setContentView(R.layout.activity_main);
        mStepText = (TextView) findViewById(R.id.step_text);
        mOrientText = (TextView) findViewById(R.id.orient_text);
        mStepView = (StepView) findViewById(R.id.step_surfaceView);
        etStationID = (EditText) findViewById(R.id.stationID);
        etExitID = (EditText) findViewById(R.id.exitID);
        etPathID = (EditText) findViewById(R.id.pathID);
        btnStart = (Button) findViewById(R.id.start);
        btnStart.setOnClickListener(this);
        btnEnd = (Button) findViewById(R.id.end);
        btnEnd.setOnClickListener(this);
        //注册其他传感器
//        MainActivity runnable = new MainActivity();
//        Thread writeFileThread = new Thread(runnable, "writeFile");
//        writeFileThread.start();
        init();
        // 注册计步监听
        mStepSensor = new StepSensorAcceleration(this, this);
        if (!mStepSensor.registerStep()) {
            Toast.makeText(this, "计步功能不可用！", Toast.LENGTH_SHORT).show();
        }
        // 注册方向监听
        mOrientSensor = new OrientSensor(this, this);
        if (!mOrientSensor.registerOrient()) {
            Toast.makeText(this, "方向功能不可用！", Toast.LENGTH_SHORT).show();
        }
        isBegin = false;
    }

    private void init(){
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        //注册加速度传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_GAME)) {
            Log.i(TAG, "加速度传感器可用！");
        } else {
            Log.i(TAG, "加速度传感器不可用！");
        }

        // 注册地磁场传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_GAME)) {
            Log.i(TAG, "地磁传感器可用！");
        } else {
            Log.i(TAG, "地磁传感器不可用！");
        }

        // 注册陀螺仪传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_GAME)) {
            Log.i(TAG, "陀螺仪传感器可用！");
        } else {
            Log.i(TAG, "陀螺仪传感器不可用！");
        }

        // 注册方向传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION),
                SensorManager.SENSOR_DELAY_GAME)) {
            Log.i(TAG, "方向传感器可用！");
        } else {
            Log.i(TAG, "方向传感器不可用！");
        }

        // 注册重力传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY),
                SensorManager.SENSOR_DELAY_GAME)) {
            Log.i(TAG, "重力传感器可用！");
        } else {
            Log.i(TAG, "重力传感器不可用！");
        }

        // 注册线性加速度传感器
        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                SensorManager.SENSOR_DELAY_GAME)) {
            Log.i(TAG, "线性加速度传感器可用！");
        } else {
            Log.i(TAG, "线性加速度传感器不可用！");
        }

//        // 注册气压传感器
//        if (mSensorManager.registerListener(sensorListener, mSensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE),
//                SensorManager.SENSOR_DELAY_GAME)) {
//            Log.i(TAG, "气压传感器可用！");
//        } else {
//            Log.i(TAG, "气压传感器不可用！");
//        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // 注销传感器监听
        mStepSensor.unregisterStep();
        mOrientSensor.unregisterOrient();
        mSensorManager.unregisterListener((SensorEventListener) this);
        try {
            fosAcce.close();
            fosGrav.close();
            fosGyro.close();
            fosPath.close();
            fosLineAcc.close();
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
            if (isBegin){
                double dt;
                if(sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
                    StringBuilder sb = new StringBuilder();
                    sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
                    sb.append(sensorEvent.values[0]+"\t");
                    sb.append(sensorEvent.values[1]+"\t");
                    sb.append(sensorEvent.values[2]+"\t\n");
                    try {
                        fosAcce.write(sb.toString().getBytes());
                        fosAcce.flush();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                if(sensorEvent.sensor.getType() == Sensor.TYPE_GYROSCOPE){
                    StringBuilder sb = new StringBuilder();
                    sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
                    sb.append(sensorEvent.values[0]+"\t");
                    sb.append(sensorEvent.values[1]+"\t");
                    sb.append(sensorEvent.values[2]+"\t\n");
                    try {
                        fosGyro.write(sb.toString().getBytes());
                        fosGyro.flush();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }

                if(sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
                    StringBuilder sb = new StringBuilder();
                    sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
                    sb.append(sensorEvent.values[0]+"\t");
                    sb.append(sensorEvent.values[1]+"\t");
                    sb.append(sensorEvent.values[2]+"\t\n");
                    try {
                        fosMagn.write(sb.toString().getBytes());
                        fosMagn.flush();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                if(sensorEvent.sensor.getType() == Sensor.TYPE_GRAVITY){
                    StringBuilder sb = new StringBuilder();
                    sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
                    sb.append(sensorEvent.values[0]+"\t");
                    sb.append(sensorEvent.values[1]+"\t");
                    sb.append(sensorEvent.values[2]+"\t\n");
                    try {
                        fosGrav.write(sb.toString().getBytes());
                        fosGrav.flush();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                if(sensorEvent.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION){
                    StringBuilder sb = new StringBuilder();
                    sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
                    sb.append(sensorEvent.values[0]+"\t");
                    sb.append(sensorEvent.values[1]+"\t");
                    sb.append(sensorEvent.values[2]+"\t\n");
                    try {
                        fosLineAcc.write(sb.toString().getBytes());
                        fosLineAcc.flush();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
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
//                if(sensorEvent.sensor.getType() == Sensor.TYPE_PRESSURE){
//                    StringBuilder sb = new StringBuilder();
//                    sb.append((sensorEvent.timestamp - globaltimestamp)*NS2S+"\t");
//                    sb.append(sensorEvent.values[0]+"\t\n");
//                    try {
//                        fosPress.write(sb.toString().getBytes());
//                        fosPath.flush();
//                    } catch (IOException e) {
//                        e.printStackTrace();
//                    }
//                }
                if (isPathWrite){
                    StringBuilder sb = new StringBuilder();
                    double timePath = System.nanoTime();
                    sb.append((timePath - globaltimestamp)*NS2S+"\t");
                    sb.append(pathX + "\t");
                    sb.append(pathY + "\t\n");
                    try {
                        fosPath.write(sb.toString().getBytes());
                        fosPath.flush();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    isPathWrite = false;
                }
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

//    @Override
//    public void run() {
//        Log.i(TAG, "Thread is alive.");
//        sensorListener = new SensorEventListener() {
//            @Override
//            public void onSensorChanged(SensorEvent sensorEvent) {
//                if (isBegin){
//                    Log.i(TAG, "Sensor data.");
//                }
//            }
//            @Override
//            public void onAccuracyChanged(Sensor sensor, int i) {
//
//            }
//        };
//
//    }

    @Override
    public void onClick(View view) {
        switch(view.getId()){
            case R.id.start:
                pathX = 500;
                pathY = 800;
                String fileName = getFileName();
                strFileDir = Environment.getExternalStorageDirectory()+ "/a_CloudNavi/"+fileName+"/";
                fosAcce = createOutputFile(strFileDir, "ACCE.txt");
                fosGyro = createOutputFile(strFileDir, "Gyro.txt");
                fosOrit = createOutputFile(strFileDir, "Orit.txt");
                fosMagn = createOutputFile(strFileDir, "Magn.txt");
                fosPress = createOutputFile(strFileDir, "Press.txt");
                fosLineAcc = createOutputFile(strFileDir, "LineAcc.txt");
                fosGrav = createOutputFile(strFileDir, "Grav.txt");
                fosPath = createOutputFile(strFileDir, "Path.txt");
                globaltimestamp = System.nanoTime();
                if(fosAcce!=null && fosMagn!=null && fosGyro!=null && fosOrit !=null && fosPath !=null){
                    btnStart.setEnabled(false);
                    btnEnd.setEnabled(true);
                }
                isBegin = true;
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
                    fosLineAcc.write("\n\n".getBytes());
                    fosLineAcc.flush();
                    fosLineAcc.close();
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
                btnStart.setEnabled(true);
        }

    }

    //Get the file name from EditTexts
    private String getFileName(){
        String fileName = null;
        String stationID = null, exitID = null, pathID= null;
        if("".equals(etStationID.getText().toString().trim()))
        {
            Toast.makeText(this, "请输入地铁站ID",Toast.LENGTH_LONG).show();
            stationID = String.valueOf(-1);
        } else {
            stationID = etStationID.getText().toString();
        }
        if("".equals(etExitID.getText().toString().trim()))
        {
            Toast.makeText(this, "请输入出入口ID",Toast.LENGTH_LONG).show();
            exitID = String.valueOf(-1);
        } else {
            exitID = etExitID.getText().toString();
        }
        if("".equals(etPathID.getText().toString().trim()))
        {
            Toast.makeText(this, "请输入路径ID",Toast.LENGTH_LONG).show();
            pathID = String.valueOf(-1);
        } else {
            pathID = etPathID.getText().toString();
        }

        if (stationID == "-1" || pathID == "-1" || exitID == "-1"){
            fileName = String.valueOf(System.currentTimeMillis());
        } else {
            fileName = String.valueOf(System.currentTimeMillis() + "_" + stationID + "_" + exitID + "_" + pathID);
        }
        return fileName;
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

}
