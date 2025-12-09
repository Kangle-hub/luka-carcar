#include "my_uvc.h"

cv::VideoCapture cap(0);
cv::Mat M;
cv::Mat static_mask;
cv::Size targetSize(160, 120);

cv::Mat frame;           // 原始彩色帧
cv::Mat frame_gray;      // 构建opencv对象 灰度
cv::Mat frame_flat;
cv::Mat Oribinary;
cv::Mat binary;
float thresh;
float bin_offset = 1.1;  // 二值化偏移量
uint8_t* gray_image;     // 指向图片数据的指针

int32 realfps = 0;

int8 my_uvc_camera_init(const char *path)
{
    cap.open(path);
    if (!cap.isOpened())
    {
        printf("find uvc camera error.\r\n");
        return -1;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, UVC_WIDTHRAW);
    cap.set(CAP_PROP_FRAME_HEIGHT, UVC_HEIGHTRAW);
    cap.set(CAP_PROP_FPS, UVC_FPSRAW);

    //此处参数需要根据摄像头更改
    Point2f srcPoints[4] = {
        Point2f(64.5, 55), // 左上-假设十字左上角坐标
        Point2f(94.5, 55), // 右上
        Point2f(101, 81),  // 右下
        Point2f(58, 81)    // 左下
    };
/*  风扇部分 后面再说
    Point2f srcPointsfan[4] = {
        Point2f(64.5, 55), // 左上-假设十字左上角坐标
        Point2f(94.5, 55), // 右上
        Point2f(101, 81),  // 右下
        Point2f(58, 81)    // 左下
    };
*/
    // 定义目标矩形区域（鸟瞰图尺寸）
    Point2f dstPoints[4] = {
        Point2f(75, 103.05), // 左上
        Point2f(85, 102.85), // 右上
        Point2f(85, 113),    // 右下
        Point2f(75, 113)     // 左下
    };
    // 计算透视变换矩阵
    M = getPerspectiveTransform(srcPoints, dstPoints);

/*  去畸变和形成映射表 初期先不考虑
    // 2. 计算校正映射表
    cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(
        cameraMatrix, distCoeffs, imageSize, 1, imageSize);
    cv::initUndistortRectifyMap(
        cameraMatrix, distCoeffs, cv::Mat(),
        newCameraMatrix, imageSize, CV_32FC1, map1, map2);

    // 修正逆透视变换矩阵的转换方式
    cv::Mat M_inv = M.inv();
    cv::Matx33d M_inv_matx(
        M_inv.at<double>(0, 0), M_inv.at<double>(0, 1), M_inv.at<double>(0, 2),
        M_inv.at<double>(1, 0), M_inv.at<double>(1, 1), M_inv.at<double>(1, 2),
        M_inv.at<double>(2, 0), M_inv.at<double>(2, 1), M_inv.at<double>(2, 2));

    // 使用原始相机参数
    const double fx_orig = cameraMatrix.at<double>(0, 0);
    const double fy_orig = cameraMatrix.at<double>(1, 1);
    const double cx_orig = cameraMatrix.at<double>(0, 2);
    const double cy_orig = cameraMatrix.at<double>(1, 2);

    // 畸变系数
    const double k1 = distCoeffs.at<double>(0);
    const double k2 = distCoeffs.at<double>(1);
    const double p1 = distCoeffs.at<double>(2);
    const double p2 = distCoeffs.at<double>(3);
    const double k3 = distCoeffs.at<double>(4);

    // 生成合并映射表（使用双精度计算）
    combined_map_x.create(targetSize, CV_32FC1);
    combined_map_y.create(targetSize, CV_32FC1);

    for (int v = 0; v < targetSize.height; ++v)
    {
        for (int u = 0; u < targetSize.width; ++u)
        {
            // 1. 逆透视变换（使用双精度计算）
            cv::Matx31d pt_homo(u, v, 1.0);
            cv::Matx31d pt_transformed = M_inv_matx * pt_homo;

            // 齐次坐标归一化
            const double w = pt_transformed(2);
            const double x_prime = pt_transformed(0) / (w != 0 ? w : 1e-6);
            const double y_prime = pt_transformed(1) / (w != 0 ? w : 1e-6);

            // 2. 转换为归一化坐标（使用原始相机参数）
            const double x_norm = (x_prime - cx_orig) / fx_orig;
            const double y_norm = (y_prime - cy_orig) / fy_orig;

            // 3. 应用畸变模型（严格遵循OpenCV公式）
            const double r2 = x_norm * x_norm + y_norm * y_norm;
            const double r4 = r2 * r2;
            const double r6 = r4 * r2;

            const double x_dist_norm = x_norm * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1 * x_norm * y_norm + p2 * (r2 + 2 * x_norm * x_norm);

            const double y_dist_norm = y_norm * (1 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2 * y_norm * y_norm) + 2 * p2 * x_norm * y_norm;

            // 4. 转换回图像坐标系
            double x_dist = x_dist_norm * fx_orig + cx_orig;
            double y_dist = y_dist_norm * fy_orig + cy_orig;

            // 边界保护
            // x_dist = std::clamp(x_dist, 0.0, imageSize.width-1.0);
            // y_dist = std::clamp(y_dist, 0.0, imageSize.height-1.0);

            combined_map_x.at<float>(v, u) = static_cast<float>(x_dist);
            combined_map_y.at<float>(v, u) = static_cast<float>(y_dist);
        }
    }
    // 新增srcPointsfan对应的映射表生成
    {
        // 1. 计算风扇区域的透视变换矩阵
        cv::Mat M_fan = cv::getPerspectiveTransform(srcPointsfan, dstPoints);
        cv::Mat M_inv_fan = M_fan.inv();
        cv::Matx33d M_inv_matx_fan(
            M_inv_fan.at<double>(0, 0), M_inv_fan.at<double>(0, 1), M_inv_fan.at<double>(0, 2),
            M_inv_fan.at<double>(1, 0), M_inv_fan.at<double>(1, 1), M_inv_fan.at<double>(1, 2),
            M_inv_fan.at<double>(2, 0), M_inv_fan.at<double>(2, 1), M_inv_fan.at<double>(2, 2));

        // 2. 初始化风扇区域的映射表
        combined_map_x_fan.create(targetSize, CV_32FC1);
        combined_map_y_fan.create(targetSize, CV_32FC1);

        // 3. 生成风扇区域的映射数据（保持计算逻辑相同）
        for (int v = 0; v < targetSize.height; ++v)
        {
            for (int u = 0; u < targetSize.width; ++u)
            {
                // 使用风扇区域的逆变换矩阵
                cv::Matx31d pt_homo(u, v, 1.0);
                cv::Matx31d pt_transformed = M_inv_matx_fan * pt_homo;

                // 后续计算步骤与原始代码完全一致...
                const double w = pt_transformed(2);
                const double x_prime = pt_transformed(0) / (w != 0 ? w : 1e-6);
                const double y_prime = pt_transformed(1) / (w != 0 ? w : 1e-6);

                const double x_norm = (x_prime - cx_orig) / fx_orig;
                const double y_norm = (y_prime - cy_orig) / fy_orig;

                // 3. 应用畸变模型（严格遵循OpenCV公式）
                const double r2 = x_norm * x_norm + y_norm * y_norm;
                const double r4 = r2 * r2;
                const double r6 = r4 * r2;

                const double x_dist_norm = x_norm * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1 * x_norm * y_norm + p2 * (r2 + 2 * x_norm * x_norm);

                const double y_dist_norm = y_norm * (1 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2 * y_norm * y_norm) + 2 * p2 * x_norm * y_norm;

                // 4. 转换回图像坐标系
                double x_dist = x_dist_norm * fx_orig + cx_orig;
                double y_dist = y_dist_norm * fy_orig + cy_orig;

                combined_map_x_fan.at<float>(v, u) = static_cast<float>(x_dist);
                combined_map_y_fan.at<float>(v, u) = static_cast<float>(y_dist);
            }
        }
    }
*/
    // 3. 创建静态遮罩
    static_mask = cv::Mat::zeros(targetSize, CV_8UC1);
    // 定义两个三角形的顶点坐标（基于160x120坐标系）
    std::vector<std::vector<cv::Point>> triangles = {
        // 右下三角形
        {cv::Point(95, 119), cv::Point(159, 33), cv::Point(159, 119)},
        // 左下三角形
        {cv::Point(0, 33), cv::Point(64, 119), cv::Point(0, 119)}};

    cv::fillPoly(static_mask, triangles, cv::Scalar(255));

    wait_image_refresh();
    return 0;
}

int8 wait_image_refresh()
{
    try
    {
        // 阻塞式等待图像刷新
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "未获取到有效图像帧" << std::endl;
            printf("没读到\n");
            return -1;
        }
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        printf("opencv坏了\n");
        return -1;
    }

    calculateFPS();

    // rgb转灰度
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

    cv::flip(frame_gray, frame_gray, -1);

/*  后面用重映射再说
    // 单次重映射完成所有处理
    if (remap_mode == 0)
    {
        cv::remap(frame_rgay, frame_flat, combined_map_x, combined_map_y,
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    }
    else
    {
        cv::remap(frame_rgay, frame_flat, combined_map_x_fan, combined_map_y_fan,
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    }
*/

    // 应用双三角形遮罩
    frame_flat.setTo(cv::Scalar(85), static_mask);

    thresh = cv::threshold(frame_gray, Oribinary, 0, 255, cv::THRESH_OTSU);
    printf("thresh  %f\n", thresh);

    // 用偏移后的阈值分别对两个图像进行二值化
    cv::threshold(frame_gray, Oribinary, thresh * bin_offset, 255, cv::THRESH_BINARY);
    cv::threshold(frame_flat, binary, thresh * bin_offset, 255, cv::THRESH_BINARY);

    // 将Oribinary的下半部分全部变成纯白
    Oribinary.rowRange(70, Oribinary.rows).setTo(255);

    // 在底部中间添加14x6白色遮罩还有三角遮罩
    int mask_width = 26;
    int mask_height = 9;
    int x = (frame_flat.cols - mask_width) / 2;
    int y = frame_flat.rows - mask_height;
    cv::rectangle(binary, cv::Rect(x, y, mask_width, mask_height), cv::Scalar(255), cv::FILLED);
    binary.setTo(cv::Scalar(85), static_mask);

    // cv对象转指针
    gray_image = reinterpret_cast<uint8_t *>(binary.ptr(0));

    return 0;
}

void calculateFPS(void)
{
    // 静态变量保持计算状态
    static int frame_counter = 0;
    static double start_time = static_cast<double>(getTickCount());

    // 每帧递增计数器
    frame_counter++;

    // 计算经过时间（秒）
    double elapsed = (getTickCount() - start_time) / getTickFrequency();

    // 每满1秒更新全局FPS
    if (elapsed >= 1.0)
    {
        realfps = frame_counter / elapsed; // 计算实际FPS
        frame_counter = 0;                 // 重置计数器
        start_time = getTickCount();       // 重置时间戳
    }
}