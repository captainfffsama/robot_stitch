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

    void resizeImg(const cv::Mat& input,cv::Mat& output,float xRate,float yRate)
    {
        cv::resize(input,output,cv::Size(),xRate,yRate);
    }

}
RobotImageStitch::RobotImageStitch(const std::string &host,float rRate):resizeFactor(rRate)
{
    aligner = std::make_shared<AlignRpc>(grpc::CreateChannel(host, grpc::InsecureChannelCredentials()));
    rM=(cv::Mat_<float>(3, 3) << rRate,0, 0, 0, rRate, 0,0, 0, 1);
    cv::invert(rM,rMInv);
}

int RobotImageStitch::operator()(const cv::Mat &img, cv::Mat &outImg) const
{
    int workFlag = 0;
    std::vector<cv::Mat> imgPatchs;
    getImgPatch(img, imgPatchs, cv::Vec3b(77, 255, 255), cv::Vec3b(35, 43, 36));

    cv::Mat imgA,imgB;
    resizeImg(imgPatchs[0],imgA,resizeFactor,resizeFactor);
    resizeImg(imgPatchs[1],imgB,resizeFactor,resizeFactor);

    // DEBUG:
    cv::imwrite("D:\\temp\\t1.jpg",imgPatchs[0]);
    cv::imwrite("D:\\temp\\t2.jpg",imgPatchs[1]);

    std::string imgAbase64 = AlignRpc::img2base64(imgA, "jpg");
    std::string imgBbase64 = AlignRpc::img2base64(imgB, "jpg");

    cv::Mat H = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
    auto flag = aligner->getHMatrix(imgAbase64, imgBbase64, H);
    if (flag != 0)
    {
        H = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
        std::cout<<"rpc count H failed"<<std::endl;
        workFlag=-1;
    }
    H=((rMInv*H)*rM);
#ifdef UNIT_TEST
    std::cout<<"H ori: "<<H<<std::endl;
#endif

    std::vector<cv::Point2f> patchACorners{cv::Point2f(0, 0), cv::Point2f(imgPatchs[0].cols - 1, 0), cv::Point2f(imgPatchs[0].cols - 1, imgPatchs[0].rows - 1), cv::Point2f(0, imgPatchs[0].rows - 1)};
    std::vector<cv::Point2f> patchACornersTrans;

    cvPointTransByH(patchACorners,H,patchACornersTrans);
    for(const auto&p:patchACornersTrans)
    {
        std::cout<<p<<std::endl;
    }

    int minx=std::numeric_limits<int>::max();
    int miny=std::numeric_limits<int>::max();
    int maxx=std::numeric_limits<int>::min();
    int maxy=std::numeric_limits<int>::min();

    for(const auto& p:patchACornersTrans)
    {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        minx=std::min(x,minx);
        miny=std::min(y,miny);
        maxx=std::max(x,maxx);
        maxy=std::max(y,maxy);
    }

#ifdef UNIT_TEST
    std::cout<<minx<<","<<miny<<","<<maxx<<","<<maxy<<std::endl;
#endif
    int tb=std::max(-miny,0);
    int lb=std::max(-minx,0);
    int rb=std::max(maxx-imgPatchs[1].cols,0);
    int bb=std::max(maxy-imgPatchs[1].rows,0);
    cv::copyMakeBorder(imgPatchs[1],outImg,tb,bb,lb,rb,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
    cv::Mat fixM=(cv::Mat_<float>(3, 3) << 1,0, lb, 0, 1, tb,0, 0, 1);
    H=fixM*H;

#ifdef UNIT_TEST
    std::cout<<"rM: "<<rM<<std::endl;
    std::cout<<"rMInv: "<<rMInv<<std::endl;
    std::cout << "H is: " << H << std::endl;
    std::cout<<"maxx: "<<maxx<<"   maxy: "<<maxy<<std::endl;
#endif
    cv::warpPerspective(imgPatchs[0],outImg,H,outImg.size());
//    cv::line(outImg,cv::Point(0,outImg.rows-imgPatchs[1].rows),cv::Point(outImg.cols-1,outImg.rows-imgPatchs[1].rows),cv::Scalar(0,0,255),10);
    imgPatchs[1].copyTo(outImg(cv::Rect(lb,tb,imgPatchs[1].cols,imgPatchs[1].rows)),cv::Mat::ones(imgPatchs[1].rows,imgPatchs[1].cols,CV_8UC1));

    return workFlag;
}

void RobotImageStitch::getImgPatch(const cv::Mat &srcImg, std::vector<cv::Mat> &imgPatchs, const cv::Vec3b &colorUpper, const cv::Vec3b &colorLower) const
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

    imgPatchs.push_back(srcImg(cv::Range(0, rowIndex - 10), cv::Range(200, colIndex - 10)).clone());
    imgPatchs.push_back(srcImg(cv::Range(rowIndex + 10, hsvSrc.rows), cv::Range(200, colIndex - 10)).clone());
}
