#ifndef ALIGNRPC_H
#define ALIGNRPC_H
#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include "opencv2/opencv.hpp"
#include "kp2d.grpc.pb.h"

#include <QImage>
#include <QBuffer>


class AlignRpc
{
public:
    AlignRpc(std::shared_ptr<grpc::Channel> channel)
        : stub_(KP2D::Kp2d::NewStub(channel)){};

    int getHMatrix(const std::string& img1base64,const std::string& img2base64,cv::Mat& H);

    static std::string img2base64(const cv::Mat& imgMat,const QString& imgExt);


private:
    std::unique_ptr<KP2D::Kp2d::Stub> stub_;
};

namespace  {
    void mat2QImage(const cv::Mat& mat, QImage& dstImg);

}
#endif // ALIGNRPC_H
