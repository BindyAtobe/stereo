# stereo
### 环境依赖
OpenCV 4.1.2-dev  
cmake 3.15
### 使用方法
1. calibrate_undistort和myCalibrate_undistort用法类似，以calibrate_undistort为例
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
2. stereo_calib
   1. 进入项目目录stereo_calib，新建一个文件夹myBuild  
      `mkdir myBuild`
   2. 将Build文件夹中的.xml文件复制到myBuild，里面存储着需要读取的图片路径信息
   3. 进入myBuild，用cmake编译
   4. 用默认参数运行程序即可  
      `./stereo_calib`
   5. 当立体校正后绘制过对极线的窗口弹出时，按空格显示下一张图片
   6. 也可以获取更多参数信息  
      `./stereo_calib  -help`
3. stereo_match_sgbm
   1. 如上用cmake编译
   2. 运行程序  
      `./stereo_match_sgbm 左图路径 右图路径 -blocksize=窗口大小 -max-disparity=视差搜索范围 -i=内参文件路径 -e=外参文件路径 -o=视差图名称`
   3. 也可以获取更多参数信息  
      `./stereo_match_sgbm  -help`
### 待办事宜
- [x] 相机基础
   - [x] 相机标定(question6)-calibrate_undistort
   - [x] 畸变校正(question7)-calibrate_undistort
   - [x] 复现张正友标定(question8)-myCalibrate_undistort
- [x] 双目基础
   - [x] 立体标定(question12)-stereo_calib
   - [x] 立体校正(question14)-stereo_calib
- [ ] 立体匹配
   - [x] SGBM(question17)-stereo_match_sgbm
### 备注
* data文件夹下是实验数据
* 我的实验结果保存在`项目文件夹/build/文件夹`中
* 如果您使用的是较老版本的OpenCV，可以访问[官方文档](https://docs.opencv.org/master/)来找到可替换的旧接口
* 您也可以配置您的IDE来使用或调试我的程序
