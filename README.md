/src这是一套实现纯惯导解算的matlab代码，算法原理见*.pdf文件

输入数据格式说明：

	为节省空间，数据按照二进制存储，为双精度
	fread(fid, count, 'double');count为每历元数据个数【Matlab读取示例】

	IMU的b-frame是前右下，数据格式为：
	GPS周秒、Gx、Gy、Gz、Ax、Ay、Az （G代表陀螺，A代表加速度计）
	陀螺和加速度计数据均为增量形式，单位分别为rad和m/s	

机械编排输出数据格式说明：

	GPS周秒、纬度、经度、高度、北向速度、东向速度、垂向速度、横滚角、俯仰角、航向角
	s        deg      m                m/s                        deg
