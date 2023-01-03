#include "alignrpc.h"

int AlignRpc::getHMatrix(const std::string& img1base64,
                         const std::string& img2base64, cv::Mat& H) {
  int flag = -1;
  KP2D::ImagePair imagePair;
  auto imgA_p = imagePair.mutable_imagea();
  auto imgB_p = imagePair.mutable_imageb();
  imgA_p->set_image(img1base64);
  imgB_p->set_image(img2base64);

  KP2D::GetEssentialMatrixReply resp;

  grpc::ClientContext context;
  auto status = stub_->getEssentialMatrix(&context, imagePair, &resp);

  if (!status.ok()) {
    std::cout << "rpc connect fail" << std::endl;
    flag = -1;
    return flag;
  }
  if (resp.status() < 0) {
    std::cout << "kp2d detect fail" << std::endl;
    flag = -1;
    return flag;
  }

  std::vector<float> tensorData;
  for (const float& i : resp.matrix().data()) {
    tensorData.push_back(i);
  }
  H = cv::Mat(3, 3, CV_32FC1, tensorData.data()).clone();
  flag = 0;
  return flag;
}

std::string AlignRpc::img2base64(const cv::Mat& imgMat, const QString& imgExt) {
  QImage img;
  mat2QImage(imgMat, img);
  QByteArray ba;
  QBuffer buf(&ba);
  buf.open(QIODevice::WriteOnly);

  img.save(&buf, imgExt.toStdString().c_str());

  QByteArray base64 = ba.toBase64();
  return QString::fromLatin1(base64).toStdString();
}

namespace {
void mat2QImage(const cv::Mat& mat, QImage& dstImg) {
  switch (mat.type()) {
    case CV_8UC1: {
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
      dstImg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step),
                      QImage::Format_Grayscale8)
                   .copy();
#else
      static QVector<QRgb> sColorTable;

      // only create our color table the first time
      if (sColorTable.isEmpty()) {
        sColorTable.resize(256);

        for (int i = 0; i < 256; ++i) {
          sColorTable[i] = qRgb(i, i, i);
        }
      }

      dstImg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step),
                      QImage::Format_Indexed8)
                   .copy();
      image.setColorTable(sColorTable);
#endif
      break;
    }
    case CV_8UC3: {
      // Copy input Mat
      const uchar* pSrc = (const uchar*)mat.data;
      // Create QImage with same dimensions as input Mat
      dstImg = QImage(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888)
                   .rgbSwapped();
      break;
    }
    case CV_8UC4: {
      // Copy input Mat
      const uchar* pSrc = (const uchar*)mat.data;
      // Create QImage with same dimensions as input Mat
      dstImg = QImage(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32)
                   .copy();
      break;
    }
    default:
      break;
  }
}

}  // namespace
