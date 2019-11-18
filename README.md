# stereo
### 环境依赖
OpenCV 4.1.2-dev  
cmake 3.15
### 使用方法
##### 以calibrate_undistort为例
1. 进入项目目录calibrate_undistort，新建一个文件夹myBuild  
   `mkdir myBuild`
2. 进入myBuild，用cmake编译  
   ```
   cd myBuild
   cmake ..
   make
   ```
3. 运行程序  
   `./calibrate_undistort  -w=6 -h=9 -oe -su ../../data/left`
4. 也可以获取更多参数信息  
   `./calibrate_undistort  -help`
### 待办事项
- [x] 相机基础
  - [x] 实现相机标定(question6)-calibrate_undistort
  - [x] 实现畸变校正(question7)-calibrate_undistort
  - [x] 复现张正友标定(question8)-myCalibrate_undistort
- [ ] 双目基础
- [ ] 立体匹配
### 备注
* data文件夹下是实验数据
* 我的实验结果保存在`项目文件夹/build/out_camera_data.yml`中
* 如果您使用的是较老版本的OpenCV，可以访问[官方文档](https://docs.opencv.org/master/)来找到可替换的旧接口
* 您也可以配置您的IDE来使用或调试我的程序
