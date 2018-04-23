# PDR数据采集软件

## PDR算法部分

参考：https://github.com/lifegh/StepOrient.git

## 传感器类型

- 加速度计
- 磁力仪
- 陀螺仪
- 重力计
- 线性加速度计
- 气压计
- 方向传感器

## 保存文件

1. 文件保存

	目录：a_IONavi/[timestamps + StationID + ExitID + PathID]/
	
	其中，timestamps是时间戳，StationID，ExitID和PathID均为用户输入ID号。
	
	文件名：
	
	- ACCE.txt // 加速度计读数
   - Gyro.txt // 陀螺仪读数
   - Orit.txt // 方向传感器读数
   - Magn.txt // 磁力仪读数
   - Press.txt // 气压计读数
   - LineAcc.txt // 线性加速度计读数
   - Grav.txt // 重力计读数
   - Path.txt // 路径读数，由PDR算法计算得到的二维坐标点，初始位置为（500，800）
   