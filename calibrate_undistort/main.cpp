#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <cctype>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

const char *usage =
        " \nexample command line for calibration from a live feed.\n"
        "   ./calibrate_undistort  -w=6 -h=9 -oe -su ../../data/left\n"
        " \n";

static void help() {
    printf("This is a camera calibration sample.\n"
           "Usage: question6&7 \n"
           "     -w=<board_width>         # 内角点个数的一个维度\n"
           "     -h=<board_height>        # 内角点个数的另一个维度\n"
           "     [-o=<out_camera_params>] # 结果输出文件\n"
           "     [-op]                    # 是否保存角点的像素坐标\n"
           "     [-oe]                    # 是否保存每张图的外参\n"

           "     [-zt]                    # 假定切向失真为0\n"
           "     [-a=<aspectRatio>]       # 固定(fx/fy)\n"
           "     [-p]                     # 固定光学中心在图像中央\n"

           "     [-su]                    # 标定后展示修正畸变后的图像\n"
           "     [input_data]             # 输入文件夹，其中存放着用来标定的图片\n"
           "\n");
    printf("\n%s", usage);
}

static double computeReprojectionErrors(
        const vector<vector<Point3f> > &objectPoints,
        const vector<vector<Point2f> > &imagePoints,
        const vector<Mat> &rvecs, const vector<Mat> &tvecs,
        const Mat &cameraMatrix, const Mat &distCoeffs,
        vector<float> &perViewErrors) {
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int) objectPoints.size(); i++) {
        //根据旋转向量、平移向量、相机矩阵、畸变参数，使3d世界坐标投影到2d坐标上，存储在imagePoint2中
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        //矩阵L2范数
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
        int n = (int) objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

static void calcChessboardCorners(Size boardSize, vector<Point3f> &corners) {
    corners.resize(0);
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            corners.push_back(Point3f(float(j), float(i), 0));
}

static bool runCalibration(vector<vector<Point2f> > imagePoints,
                           Size imageSize, Size boardSize, float aspectRatio,
                           int flags, Mat &cameraMatrix, Mat &distCoeffs,
                           vector<Mat> &rvecs, vector<Mat> &tvecs,
                           vector<float> &reprojErrs,
                           double &totalAvgErr) {
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (flags & CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    //计算各角点在世界坐标系的坐标
    calcChessboardCorners(boardSize, objectPoints[0]);
    vector<Point3f> tmpObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms;
    //计算内参、外参、畸变参数  *********重要***********
    rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, -1,
                            cameraMatrix, distCoeffs, rvecs, tvecs, noArray(),
                            flags | CALIB_FIX_K3 | CALIB_USE_LU);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), tmpObjPoints);
    //计算误差
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}


static void saveCameraParams(const string &filename,
                             Size imageSize, Size boardSize,
                             float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs,
                             const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                             const vector<float> &reprojErrs,
                             const vector<vector<Point2f> > &imagePoints,
                             double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nframes" << (int) std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;

    if (flags & CALIB_FIX_ASPECT_RATIO)
        fs << "aspectRatio" << aspectRatio;

    if (flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "avg_reprojection_error" << totalAvgErr;
    if (!reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if (!rvecs.empty() && !tvecs.empty()) {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int) rvecs.size(), 6, rvecs[0].type());
        for (int i = 0; i < (int) rvecs.size(); i++) {
            Mat r = bigmat(Range(i, i + 1), Range(0, 3));
            Mat t = bigmat(Range(i, i + 1), Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if (!imagePoints.empty()) {
        Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int) imagePoints.size(); i++) {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

static bool runAndSave(const string &outputFilename,
                       const vector<vector<Point2f> > &imagePoints,
                       Size imageSize, Size boardSize,
                       float aspectRatio, int flags, Mat &cameraMatrix,
                       Mat &distCoeffs, bool writeExtrinsics, bool writePoints) {
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize,
                             aspectRatio, flags, cameraMatrix, distCoeffs,
                             rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.7f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if (ok)
        saveCameraParams(outputFilename, imageSize,
                         boardSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr);
    return ok;
}


int main(int argc, char **argv) {
    Size boardSize, imageSize;
    float aspectRatio = 1;
    Mat cameraMatrix, distCoeffs;
    string outputFilename;
    string inputFilename = "";

    int i;
    bool writeExtrinsics, writePoints;
    int flags = 0;
    bool showUndistorted;
    vector<vector<Point2f> > imagePoints;
    vector<string> imageList;

    cv::CommandLineParser parser(argc, argv,
                                 "{help ||}{w||}{h||}{o|out_camera_data.yml|}"
                                 "{op||}{oe||}{zt||}{a||}{p||}{su||}{@input_data|0|}");
    if (parser.has("help")) {
        help();
        return 0;
    }
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    writePoints = parser.has("op");
    writeExtrinsics = parser.has("oe");

    if (parser.has("a")) {
        //固定fx和fy的比值
        flags |= CALIB_FIX_ASPECT_RATIO;
        aspectRatio = parser.get<float>("a");
    }
    if (parser.has("zt"))
        //忽略切向畸变
        flags |= CALIB_ZERO_TANGENT_DIST;
    if (parser.has("p"))
        //固定光学中心
        flags |= CALIB_FIX_PRINCIPAL_POINT;

    if (parser.has("o"))
        outputFilename = parser.get<string>("o");
    showUndistorted=parser.has("su");
    inputFilename = parser.get<string>("@input_data");
    int winSize = 11;
    if (!parser.check()) {
        help();
        parser.printErrors();
        return -1;
    }
    if (aspectRatio <= 0)
        return printf("Invalid aspect ratio\n"), -1;
    if (boardSize.width <= 0)
        return fprintf(stderr, "Invalid board width\n"), -1;
    if (boardSize.height <= 0)
        return fprintf(stderr, "Invalid board height\n"), -1;

    if (!inputFilename.empty()) {
        //imageList存放了图片相对或绝对路径
        glob(inputFilename, imageList);
    }
    if (imageList.size() <= 3)
        return printf("Invalid number of images\n"), -1;

    namedWindow("Image View", 1);

    //====================================主要代码========================================
    for (i = 0; i < (int) imageList.size(); i++) {
        Mat view, viewGray;
        view = imread(imageList[i], 1);//读取图像 *******关键********
        if (view.empty()) {
            continue;
        }
        imageSize = view.size();
        vector<Point2f> pointbuf;
        cvtColor(view, viewGray, COLOR_BGR2GRAY);//转为灰度图

        bool found;
        //寻找角点，保存至pointbuf  *********关键***********
        found = findChessboardCorners(view, boardSize, pointbuf,
                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            //亚像素级角点，提高精度
            cornerSubPix(viewGray, pointbuf, Size(winSize, winSize),
                         Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
            imagePoints.push_back(pointbuf);
            //在view上标出角点
            drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
        }
        //--------------实现右下角标注 这是当前处理的第几张图片-------------
        string msg = "100/100";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);
        msg = format("%d/%d", (int) imagePoints.size(), (int) imageList.size());
        putText(view, msg, textOrigin, 1, 1, Scalar(0, 0, 255));
        //-------------------------显示标好角点的图片，并检测用户输入--------------------------
        imshow("Image View", view);//在窗口显示view
        char key = (char) waitKey(500);//延时
        if (key == 27 || key == 'q' || key == 'Q')
            break;
    }
    //图片全处理完了，标定并保存
    runAndSave(outputFilename, imagePoints, imageSize,
               boardSize, aspectRatio,
               flags, cameraMatrix, distCoeffs,
               writeExtrinsics, writePoints);
    //======================================================================================
    if (showUndistorted) {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                imageSize, CV_16SC2, map1, map2);

        for (i = 0; i < (int) imageList.size(); i++) {
            view = imread(imageList[i], 1);
            if (view.empty())
                continue;
            //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char key = (char) waitKey(500);
            if (key == 27 || key == 'q' || key == 'Q')
                break;
        }
    }
    return 0;
}
