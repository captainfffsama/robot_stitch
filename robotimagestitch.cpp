#include "robotimagestitch.h"

namespace
{
    bool isSameColor(const cv::Vec3b &betest, const cv::Vec3b &upper, const cv::Vec3b &lower)
    {
        return (betest[0] < upper[0]) && (betest[0] > lower[0]) && (betest[1] < upper[1]) && (betest[1] > lower[1]) && (betest[2] < upper[2]) && (betest[2] > lower[2]);
    }

    void cvPointTransByH(const std::vector<cv::Point2f> &srcPoint, const cv::Mat &mat, std::vector<cv::Point2f> &dstPoint)
    {
        if (mat.rows == 2)
        {
            cv::transform(srcPoint, dstPoint, mat);
        }
        if (mat.rows == 3)
        {
            cv::perspectiveTransform(srcPoint, dstPoint, mat);
        }
        else
        {
            dstPoint = srcPoint;
        }
    }
}
RobotImageStitch::RobotImageStitch(const std::string &host)
{
    aligner = std::make_shared<AlignRpc>(grpc::CreateChannel(host, grpc::InsecureChannelCredentials()));
}

bool RobotImageStitch::operator()(const cv::Mat &img, cv::Mat &outImg)
{
    bool workFlag = true;
    std::vector<cv::Mat> imgPatchs;
    getImgPatch(img, imgPatchs, cv::Vec3b(77, 255, 255), cv::Vec3b(35, 43, 36));

    std::string imgAbase64 = AlignRpc::img2base64(imgPatchs[0], "jpg");
    std::string imgBbase64 = AlignRpc::img2base64(imgPatchs[1], "jpg");

    cv::Mat H = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
    auto flag = aligner->getHMatrix(imgAbase64, imgBbase64, H);
    if (flag != 0)
    {
        H = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
    }
#ifdef UNIT_TEST
    std::cout << "H is: " << H << std::endl;
#endif

    std::vector<cv::Point2f> patchBCorners{cv::Point2f(0, 0), cv::Point2f(imgPatchs[1].cols - 1, 0), cv::Point2f(imgPatchs[1].cols - 1, imgPatchs[1].rows - 1), cv::Point2f(0, imgPatchs[1].rows - 1)};
    std::vector<cv::Point2f> patchBCornersTrans;

    cvPointTransByH(patchBCorners,H,patchBCornersTrans);

    int minx=std::numeric_limits<int>::max();
    int miny=std::numeric_limits<int>::max();
    int maxx=std::numeric_limits<int>::min();
    int maxy=std::numeric_limits<int>::min();

    for(const auto& p:patchBCornersTrans)
    {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        minx=std::min(x,minx);
        miny=std::min(y,miny);
        maxx=std::max(x,maxx);
        maxy=std::max(y,maxy);
    }
    if(minx<0)
    {maxx-=minx;}
    if(miny<0)
    {maxy-=miny;}

    maxx=std::max(maxx,imgPatchs[0].cols);
    maxy=std::max(maxy,imgPatchs[0].rows);

    cv::warpPerspective(imgPatchs[1],outImg,H,cv::Size(maxx,maxy));
    outImg(cv::Range(0,imgPatchs[1].rows-1),cv::Range(0,imgPatchs[1].cols-1))=imgPatchs[1];

    return workFlag;
}

void RobotImageStitch::getImgPatch(const cv::Mat &srcImg, std::vector<cv::Mat> &imgPatchs, const cv::Vec3b &colorUpper, const cv::Vec3b &colorLower)
{
    cv::Mat hsvSrc;
    cv::cvtColor(srcImg, hsvSrc, cv::COLOR_BGR2HSV);

    std::vector<int> colCount(hsvSrc.cols, 0);

    hsvSrc.forEach<cv::Vec3b>([&colCount, &colorUpper, &colorLower](cv::Vec3b &pixel, const int position[]) -> void
                              {
        if(isSameColor(pixel,colorUpper,colorLower))
        {
            colCount[position[1]]++;
        } });
    auto coliter = std::max_element(colCount.begin(), colCount.end(), [](int i, int j) -> bool
                                    { return i < j; });

    int colIndex = coliter - colCount.begin();

    std::vector<int> rowCount(hsvSrc.rows, 0);
    hsvSrc(cv::Range(0, hsvSrc.rows), cv::Range(0, colIndex)).forEach<cv::Vec3b>([&rowCount, &colorUpper, &colorLower](cv::Vec3b &pixel, const int position[]) -> void
                                                                                 {
        if(isSameColor(pixel,colorUpper,colorLower))
        {
            rowCount[position[0]]++;
        } });

    auto rowiter = std::max_element(rowCount.begin(), rowCount.end(), [](int i, int j) -> bool
                                    { return i < j; });
    int rowIndex = rowiter - rowCount.begin();

    imgPatchs.push_back(srcImg(cv::Range(0, rowIndex - 5), cv::Range(0, colIndex - 5)).clone());
    imgPatchs.push_back(srcImg(cv::Range(rowIndex + 5, hsvSrc.rows), cv::Range(0, colIndex - 5)).clone());
}
