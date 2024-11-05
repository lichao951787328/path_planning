#include <opencv2/opencv.hpp>

int main() 
{
    cv::Mat binaryImage = cv::imread("path_to_binary_image.png", cv::IMREAD_GRAYSCALE);

    if (binaryImage.empty()) {
        std::cerr << "Error loading image" << std::endl;
        return -1;
    }

    // 确保图像为二值图像（黑白）
    cv::threshold(binaryImage, binaryImage, 128, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> hulls(contours.size());

    // cv::RETR_EXTERNAL 表示只提取最外层的轮廓，忽略内嵌的轮廓。
    // cv::CHAIN_APPROX_SIMPLE 表示只保存轮廓的角点，去掉冗余的点（如沿一条直线的所有中间点）。这样可以减少存储点的数量，从而节省内存。
    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) 
    {
        // 计算给定轮廓的凸包，输入的轮廓，即一系列点的集合，输出的凸包点集。hulls[i] 是用于存储与 contours[i] 对应的凸包的点的集合。凸包是包含所有轮廓点的最小凸多边形。
        cv::convexHull(contours[i], hulls[i]);
    }

    for (size_t i = 0; i < contours.size(); i++) 
    {
        // 计算多边形逼近
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 5.0, true);

        // 标记凸点和凹点
        for (size_t j = 0; j < approx.size(); j++) 
        {
            if (cv::pointPolygonTest(hulls[i], approx[j], false) < 0) 
            {
                // approx[j] 是凹点
            }
        }
    }

    cv::Mat dist;
    cv::distanceTransform(binaryImage, dist, cv::DIST_L2, 5);
    cv::normalize(dist, dist, 0, 1.0, cv::NORM_MINMAX);

    // 找到局部极大值点
    cv::threshold(dist, dist, 0.4, 1.0, cv::THRESH_BINARY);
    cv::Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    // 找到标记
    std::vector<std::vector<cv::Point>> contours_markers;
    cv::findContours(dist_8u, contours_markers, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat markers = cv::Mat::zeros(dist.size(), CV_32SC1);
    for (size_t i = 0; i < contours_markers.size(); i++) 
    {
        cv::drawContours(markers, contours_markers, static_cast<int>(i), cv::Scalar(static_cast<int>(i) + 1), -1);
    }

    cv::watershed(binaryImage, markers);

    // 生成结果图像
    cv::Mat segmentedRegions;
    markers.convertTo(segmentedRegions, CV_8U);

    cv::imshow("Convex Segmented Image", segmentedRegions);
    cv::waitKey(0);
    cv::imwrite("convex_segmented_image.png", segmentedRegions);


    return 0;
}
