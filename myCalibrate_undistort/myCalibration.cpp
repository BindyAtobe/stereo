#include "myCalibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/core/cvdef.h>
#include "copyFromOpenCV/calib3d_c_api.h"

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

void calv_t(const Mat &H, int i, int j, Mat &dst);

void myInitParams2D(const Mat &matM,
                    const Mat &matm,
                    const Mat &npoints,
                    Matx33d &cameraMatrix,
                    Mat &matRvec, Mat &mattvec);

double myCalibrateCamera2Internal(const Mat &objectPoints,
                                  const Mat &imagePoints,
                                  const Mat &npoints,
                                  Mat &cameraMatrix, Mat &distCoeffs,
                                  Mat &rvecM, Mat &tvecM);


double myCalibrateCamera(InputArrayOfArrays _objectPoints,
                         InputArrayOfArrays _imagePoints,
                         InputOutputArray _cameraMatrix, InputOutputArray _distCoeffs,
                         OutputArrayOfArrays _rvecs, OutputArrayOfArrays _tvecs) {
    int nimages = (int) _objectPoints.total(), ni, total = 0;
    CV_Assert(nimages >= 3);
    for (int i = 0; i < nimages; ++i) {
        Mat objpt = _objectPoints.getMat(i);
        ni = objpt.checkVector(3, CV_32F);
        Mat imgpt = _imagePoints.getMat(i);
        int ni1 = imgpt.checkVector(2, CV_32F);
        CV_Assert(ni == ni1);//检查每幅图3d点和2d点个数是否对应相同
        total += ni;//得到角点总数
    }
    Mat objectPoints(1, total, CV_32FC3),
            imagePoints(1, total, CV_32FC2),
            npoints(1, nimages, CV_32S);
    Point3f *objPtData = objectPoints.ptr<Point3f>();
    Point2f *imgPtData = imagePoints.ptr<Point2f>();
    for (int i = 0, j = 0; i < nimages; ++i, j += ni) {
        Mat objpt = _objectPoints.getMat(i);
        Mat imgpt = _imagePoints.getMat(i);
        ni = objpt.checkVector(3, CV_32F);
        npoints.at<int>(i) = ni;
        for (int n = 0; n < ni; ++n) {
            objPtData[j + n] = objpt.ptr<Point3f>()[n];
            imgPtData[j + n] = imgpt.ptr<Point2f>()[n];
        }
    }

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    Mat rvecM(nimages, 3, CV_64F), tvecM(nimages, 3, CV_64F);
    double reprojErr = myCalibrateCamera2Internal(objectPoints, imagePoints, npoints,
                                                  cameraMatrix, distCoeffs,
                                                  rvecM, tvecM);
    _rvecs.create(nimages, 1, CV_64FC3);
    _tvecs.create(nimages, 1, CV_64FC3);
    for (int i = 0; i < nimages; i++) {
        _rvecs.create(3, 1, CV_64F, i, true);
        Mat rv = _rvecs.getMat(i);
        memcpy(rv.ptr(), rvecM.ptr(i), 3 * sizeof(double));
        _tvecs.create(3, 1, CV_64F, i, true);
        Mat tv = _tvecs.getMat(i);
        memcpy(tv.ptr(), tvecM.ptr(i), 3 * sizeof(double));
    }
    cameraMatrix.copyTo(_cameraMatrix);
    distCoeffs.copyTo(_distCoeffs);
    return reprojErr;
}

double myCalibrateCamera2Internal(const Mat &objectPoints,
                                  const Mat &imagePoints,
                                  const Mat &npoints,
                                  Mat &cameraMatrix, Mat &distCoeffs,
                                  Mat &rvecM, Mat &tvecM) {
    double reprojErr = 0;
    int nimages = npoints.rows * npoints.cols, maxPoints = 0, ni, total = 0, nparams, npstep, cn;
    const int NINTRINSIC = CV_CALIB_NINTRINSIC;
    nparams = NINTRINSIC + nimages * 6;//总参数
    for (int i = 0; i < nimages; i++) {
        ni = npoints.at<int>(i);
        if (ni < 4) {
            CV_Error_(cv::Error::StsOutOfRange, ("The number of points in the view #%d is < 4", i));
        }
        maxPoints = MAX(maxPoints, ni);
        total += ni;
    }

    Mat matm;
    convertPointsToHomogeneous(imagePoints, matm);
    matm = matm.t();
    Mat matM;
    objectPoints.convertTo(matM, CV_64FC3);
    for (int i = 0; i < total; i++) {
        matM.at<Point3d>(i).z = 1.;
    }
    Mat allErrors(1, total, CV_64FC2);

    Matx33d A;
    A(1, 0) = A(2, 0) = A(2, 1) = 0.;
    A(2, 2) = 1.;

    Mat matRvec, mattvec;
    //初始化内参和外参
    myInitParams2D(matM, matm, npoints, A, matRvec, mattvec);
    cout << "=======Before optimize============" << endl
         << "--内参--\n" << A << endl
         << "--旋转向量--\n" << matRvec << endl
         << "--平移向量--\n" << mattvec << endl;
    A(0, 1) = 0;//gamma当做0
    //优化
    double k[14] = {0};
    Mat matk(1, 8, CV_64F);
    CvMat _k = cvMat(distCoeffs.rows, distCoeffs.cols, CV_64F, k);
    CvMat _A = cvMat(3, 3, CV_64F, A.val);
    CvLevMarq solver(nparams, 0, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01));
    Mat _Ji(maxPoints * 2, NINTRINSIC, CV_64FC1, Scalar(0));
    Mat _Je(maxPoints * 2, 6, CV_64FC1);
    Mat _err(maxPoints * 2, 1, CV_64FC1);
    const bool allocJo = (solver.state == CvLevMarq::CALC_J);
    Mat _Jo = allocJo ? Mat(maxPoints * 2, maxPoints * 3, CV_64FC1, Scalar(0)) : Mat();
    {
        //设初值
        double *param = solver.param->data.db;
        uchar *mask = solver.mask->data.ptr;
        param[0] = A(0, 0);
        param[1] = A(1, 1);//fx fy
        param[2] = A(0, 2);
        param[3] = A(1, 2);//u0 v0
        //畸变系数
        copy(k, k + 14, param + 4);
        //不考虑径向畸变--mask[6],mask[7]   不考虑k3 k4 k5--mask[8]..
        for (int i = 8; i < 18; ++i) {
            mask[i] = 0;
        }

        int pos = NINTRINSIC;
        for (int i = 0; i < nimages; ++i) {
            for (int j = 0; j < 3; ++j) {
                param[pos++] = matRvec.at<double>(i, j);
            }
            for (int j = 0; j < 3; ++j) {
                param[pos++] = mattvec.at<double>(i, j);
            }
        }
    }

    for (;;) {
        const CvMat *_param = 0;
        CvMat *_JtJ = 0, *_JtErr = 0;
        double *_errNorm = 0;
        bool proceed = solver.updateAlt(_param, _JtJ, _JtErr, _errNorm);
        double *param = solver.param->data.db;
        bool calcJ = solver.state == CvLevMarq::CALC_J;

        A(0, 0) = param[0];
        A(1, 1) = param[1];
        A(0, 2) = param[2];
        A(1, 2) = param[3];
        copy(param + 4, param + 4 + 14, k);

        if (!proceed)
            break;

        reprojErr = 0;

        for (int i = 0, pos = 0; i < nimages; i++, pos += ni) {
            CvMat _ri, _ti;
            ni = npoints.at<int>(i);

            cvGetRows(solver.param, &_ri, NINTRINSIC + i * 6, NINTRINSIC + i * 6 + 3);
            cvGetRows(solver.param, &_ti, NINTRINSIC + i * 6 + 3, NINTRINSIC + i * 6 + 6);

            CvMat _Mi = cvMat(objectPoints.colRange(pos, pos + ni));
            CvMat _mi = cvMat(imagePoints.colRange(pos, pos + ni));

            _Je.resize(ni * 2);
            _Ji.resize(ni * 2);
            _err.resize(ni * 2);
            _Jo.resize(ni * 2);

            CvMat _mp = cvMat(_err.reshape(2, 1));

            if (calcJ) {
                CvMat _dpdr = cvMat(_Je.colRange(0, 3));
                CvMat _dpdt = cvMat(_Je.colRange(3, 6));
                CvMat _dpdf = cvMat(_Ji.colRange(0, 2));
                CvMat _dpdc = cvMat(_Ji.colRange(2, 4));
                CvMat _dpdk = cvMat(_Ji.colRange(4, NINTRINSIC));
                CvMat _dpdo = _Jo.empty() ? CvMat() : cvMat(_Jo.colRange(0, ni * 3));

                cvProjectPoints2Internal(&_Mi, &_ri, &_ti, &_A, &_k, &_mp, &_dpdr, &_dpdt,
                                         &_dpdf,
                                         &_dpdc, &_dpdk,
                                         (_Jo.empty()) ? nullptr : &_dpdo,
                                         0);
            } else
                cvProjectPoints2(&_Mi, &_ri, &_ti, &_A, &_k, &_mp);

            cvSub(&_mp, &_mi, &_mp);

            if (calcJ) {
                Mat JtJ(cvarrToMat(_JtJ)), JtErr(cvarrToMat(_JtErr));

                // see HZ: (A6.14) for details on the structure of the Jacobian
                JtJ(Rect(0, 0, NINTRINSIC, NINTRINSIC)) += _Ji.t() * _Ji;
                JtJ(Rect(NINTRINSIC + i * 6, NINTRINSIC + i * 6, 6, 6)) = _Je.t() * _Je;
                JtJ(Rect(NINTRINSIC + i * 6, 0, 6, NINTRINSIC)) = _Ji.t() * _Je;

                JtErr.rowRange(0, NINTRINSIC) += _Ji.t() * _err;
                JtErr.rowRange(NINTRINSIC + i * 6, NINTRINSIC + (i + 1) * 6) = _Je.t() * _err;
            }

            double viewErr = norm(_err, NORM_L2SQR);

            reprojErr += viewErr;
        }
        if (_errNorm)
            *_errNorm = reprojErr;
    }

    // 4. 保存结果
    Mat(A).copyTo(cameraMatrix);
    cvarrToMat(&_k).copyTo(distCoeffs);
    for (int i = 0, pos = 0; i < nimages; i++) {
        CvMat src, dst;

        src = cvMat(3, 1, CV_64F, solver.param->data.db + NINTRINSIC + i * 6);
        dst = cvMat(3, 1, CV_64F, rvecM.data + rvecM.step * i);
        cvConvert(&src, &dst);
        src = cvMat(3, 1, CV_64F, solver.param->data.db + NINTRINSIC + i * 6 + 3);
        dst = cvMat(3, 1, CV_64F, tvecM.data + tvecM.step * i);
        cvConvert(&src, &dst);
    }
    return sqrt(reprojErr / total);
}

void myInitParams2D(const Mat &matM,
                    const Mat &matm,
                    const Mat &npoints,
                    Matx33d &A,
                    Mat &matRvec, Mat &mattvec) {
    int nimages = npoints.rows * npoints.cols, ni;
    Mat matV;
    Matx61d b;
    vector<Mat> vecH;
    //先求H，再求V，解出b
    for (int i = 0, pos = 0; i < nimages; ++i, pos += ni) {
        ni = npoints.at<int>(i);
        Mat matMi = matM.colRange(pos, pos + ni);
        Mat matmi = matm.colRange(pos, pos + ni);
        vecH.push_back(findHomography(matMi, matmi));
        Mat v12_t(1, 6, CV_64F), v11_t(1, 6, CV_64F), v22_t(1, 6, CV_64F);
        calv_t(vecH[i], 1, 2, v12_t);
        calv_t(vecH[i], 1, 1, v11_t);
        calv_t(vecH[i], 2, 2, v22_t);
        matV.push_back(v12_t);
        matV.push_back(Mat(v11_t - v22_t));
    }
    SVD::solveZ(matV, b);
    //根据b初始化内参
    double v0 = (b(1) * b(3) - b(0) * b(4)) / (b(0) * b(2) - b(1) * b(1));
    double lambda = b(5) - (b(3) * b(3) + v0 * (b(1) * b(3) - b(0) * b(4))) / b(0);
    double alpha = sqrt(lambda / b(0));
    double beta = sqrt(lambda * b(0) / (b(0) * b(2) - b(1) * b(1)));
    double gamma = -b(1) * alpha * alpha * beta / lambda;
    double u0 = gamma * v0 / alpha - b(3) * alpha * alpha / lambda;
    A(0, 0) = alpha;
    A(0, 1) = gamma;//gamma常当作0
    A(0, 2) = u0;
    A(1, 1) = beta;
    A(1, 2) = v0;
    //初始化外参
    for (int i = 0; i < nimages; ++i) {
        Mat A_1 = Mat(A.inv()), Hi = vecH[i];
        double lambda1 = 1 / norm(A_1 * Hi.col(0));
        double lambda2 = 1 / norm(A_1 * Hi.col(1));
        double lambdat = (lambda1 + lambda2) / 2;
        Mat r1 = lambda1 * A_1 * Hi.col(0);
        Mat r2 = lambda2 * A_1 * Hi.col(1);
        Mat r3 = r1.cross(r2);
        Mat Ri(3, 3, CV_64F);
        r1.copyTo(Ri.col(0));
        r2.copyTo(Ri.col(1));
        r3.copyTo(Ri.col(2));
        Mat rveci;
        Rodrigues(Ri, rveci);
        matRvec.push_back(rveci.t());
        mattvec.push_back((lambdat * A_1 * Hi.col(2)).t());
    }
}

void calv_t(const Mat &H, int i, int j, Mat &dst) {
    //注意下标
    --i;
    --j;
    dst.at<double>(0) = H.at<double>(0, i) * H.at<double>(0, j);
    dst.at<double>(1) = H.at<double>(0, i) * H.at<double>(1, j) + H.at<double>(1, i) * H.at<double>(0, j);
    dst.at<double>(2) = H.at<double>(1, i) * H.at<double>(1, j);
    dst.at<double>(3) = H.at<double>(2, i) * H.at<double>(0, j) + H.at<double>(0, i) * H.at<double>(2, j);
    dst.at<double>(4) = H.at<double>(2, i) * H.at<double>(1, j) + H.at<double>(1, i) * H.at<double>(2, j);
    dst.at<double>(5) = H.at<double>(2, i) * H.at<double>(2, j);
}