// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: kp2d.proto

#include "kp2d.pb.h"
#include "kp2d.grpc.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/channel_interface.h>
#include <grpcpp/impl/codegen/client_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/rpc_service_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/sync_stream.h>
namespace KP2D {

static const char* Kp2d_method_names[] = {
  "/KP2D.Kp2d/align",
  "/KP2D.Kp2d/getEssentialMatrix",
  "/KP2D.Kp2d/getKPPosDescScore",
  "/KP2D.Kp2d/getKPPosDescScoreMap",
};

std::unique_ptr< Kp2d::Stub> Kp2d::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< Kp2d::Stub> stub(new Kp2d::Stub(channel));
  return stub;
}

Kp2d::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel)
  : channel_(channel), rpcmethod_align_(Kp2d_method_names[0], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_getEssentialMatrix_(Kp2d_method_names[1], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_getKPPosDescScore_(Kp2d_method_names[2], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_getKPPosDescScoreMap_(Kp2d_method_names[3], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status Kp2d::Stub::align(::grpc::ClientContext* context, const ::KP2D::ImagePair& request, ::KP2D::AlignReply* response) {
  return ::grpc::internal::BlockingUnaryCall< ::KP2D::ImagePair, ::KP2D::AlignReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_align_, context, request, response);
}

void Kp2d::Stub::experimental_async::align(::grpc::ClientContext* context, const ::KP2D::ImagePair* request, ::KP2D::AlignReply* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::KP2D::ImagePair, ::KP2D::AlignReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_align_, context, request, response, std::move(f));
}

void Kp2d::Stub::experimental_async::align(::grpc::ClientContext* context, const ::KP2D::ImagePair* request, ::KP2D::AlignReply* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_align_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::KP2D::AlignReply>* Kp2d::Stub::PrepareAsyncalignRaw(::grpc::ClientContext* context, const ::KP2D::ImagePair& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::KP2D::AlignReply, ::KP2D::ImagePair, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_align_, context, request);
}

::grpc::ClientAsyncResponseReader< ::KP2D::AlignReply>* Kp2d::Stub::AsyncalignRaw(::grpc::ClientContext* context, const ::KP2D::ImagePair& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncalignRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status Kp2d::Stub::getEssentialMatrix(::grpc::ClientContext* context, const ::KP2D::ImagePair& request, ::KP2D::GetEssentialMatrixReply* response) {
  return ::grpc::internal::BlockingUnaryCall< ::KP2D::ImagePair, ::KP2D::GetEssentialMatrixReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_getEssentialMatrix_, context, request, response);
}

void Kp2d::Stub::experimental_async::getEssentialMatrix(::grpc::ClientContext* context, const ::KP2D::ImagePair* request, ::KP2D::GetEssentialMatrixReply* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::KP2D::ImagePair, ::KP2D::GetEssentialMatrixReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_getEssentialMatrix_, context, request, response, std::move(f));
}

void Kp2d::Stub::experimental_async::getEssentialMatrix(::grpc::ClientContext* context, const ::KP2D::ImagePair* request, ::KP2D::GetEssentialMatrixReply* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_getEssentialMatrix_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::KP2D::GetEssentialMatrixReply>* Kp2d::Stub::PrepareAsyncgetEssentialMatrixRaw(::grpc::ClientContext* context, const ::KP2D::ImagePair& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::KP2D::GetEssentialMatrixReply, ::KP2D::ImagePair, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_getEssentialMatrix_, context, request);
}

::grpc::ClientAsyncResponseReader< ::KP2D::GetEssentialMatrixReply>* Kp2d::Stub::AsyncgetEssentialMatrixRaw(::grpc::ClientContext* context, const ::KP2D::ImagePair& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncgetEssentialMatrixRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status Kp2d::Stub::getKPPosDescScore(::grpc::ClientContext* context, const ::KP2D::ImgAndThr& request, ::KP2D::GetKPPosDescScoreReply* response) {
  return ::grpc::internal::BlockingUnaryCall< ::KP2D::ImgAndThr, ::KP2D::GetKPPosDescScoreReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_getKPPosDescScore_, context, request, response);
}

void Kp2d::Stub::experimental_async::getKPPosDescScore(::grpc::ClientContext* context, const ::KP2D::ImgAndThr* request, ::KP2D::GetKPPosDescScoreReply* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::KP2D::ImgAndThr, ::KP2D::GetKPPosDescScoreReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_getKPPosDescScore_, context, request, response, std::move(f));
}

void Kp2d::Stub::experimental_async::getKPPosDescScore(::grpc::ClientContext* context, const ::KP2D::ImgAndThr* request, ::KP2D::GetKPPosDescScoreReply* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_getKPPosDescScore_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::KP2D::GetKPPosDescScoreReply>* Kp2d::Stub::PrepareAsyncgetKPPosDescScoreRaw(::grpc::ClientContext* context, const ::KP2D::ImgAndThr& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::KP2D::GetKPPosDescScoreReply, ::KP2D::ImgAndThr, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_getKPPosDescScore_, context, request);
}

::grpc::ClientAsyncResponseReader< ::KP2D::GetKPPosDescScoreReply>* Kp2d::Stub::AsyncgetKPPosDescScoreRaw(::grpc::ClientContext* context, const ::KP2D::ImgAndThr& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncgetKPPosDescScoreRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status Kp2d::Stub::getKPPosDescScoreMap(::grpc::ClientContext* context, const ::KP2D::ImgAndThr& request, ::KP2D::GetKPPosDescScoreMapReply* response) {
  return ::grpc::internal::BlockingUnaryCall< ::KP2D::ImgAndThr, ::KP2D::GetKPPosDescScoreMapReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_getKPPosDescScoreMap_, context, request, response);
}

void Kp2d::Stub::experimental_async::getKPPosDescScoreMap(::grpc::ClientContext* context, const ::KP2D::ImgAndThr* request, ::KP2D::GetKPPosDescScoreMapReply* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::KP2D::ImgAndThr, ::KP2D::GetKPPosDescScoreMapReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_getKPPosDescScoreMap_, context, request, response, std::move(f));
}

void Kp2d::Stub::experimental_async::getKPPosDescScoreMap(::grpc::ClientContext* context, const ::KP2D::ImgAndThr* request, ::KP2D::GetKPPosDescScoreMapReply* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_getKPPosDescScoreMap_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::KP2D::GetKPPosDescScoreMapReply>* Kp2d::Stub::PrepareAsyncgetKPPosDescScoreMapRaw(::grpc::ClientContext* context, const ::KP2D::ImgAndThr& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::KP2D::GetKPPosDescScoreMapReply, ::KP2D::ImgAndThr, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_getKPPosDescScoreMap_, context, request);
}

::grpc::ClientAsyncResponseReader< ::KP2D::GetKPPosDescScoreMapReply>* Kp2d::Stub::AsyncgetKPPosDescScoreMapRaw(::grpc::ClientContext* context, const ::KP2D::ImgAndThr& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncgetKPPosDescScoreMapRaw(context, request, cq);
  result->StartCall();
  return result;
}

Kp2d::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      Kp2d_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< Kp2d::Service, ::KP2D::ImagePair, ::KP2D::AlignReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](Kp2d::Service* service,
             ::grpc::ServerContext* ctx,
             const ::KP2D::ImagePair* req,
             ::KP2D::AlignReply* resp) {
               return service->align(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      Kp2d_method_names[1],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< Kp2d::Service, ::KP2D::ImagePair, ::KP2D::GetEssentialMatrixReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](Kp2d::Service* service,
             ::grpc::ServerContext* ctx,
             const ::KP2D::ImagePair* req,
             ::KP2D::GetEssentialMatrixReply* resp) {
               return service->getEssentialMatrix(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      Kp2d_method_names[2],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< Kp2d::Service, ::KP2D::ImgAndThr, ::KP2D::GetKPPosDescScoreReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](Kp2d::Service* service,
             ::grpc::ServerContext* ctx,
             const ::KP2D::ImgAndThr* req,
             ::KP2D::GetKPPosDescScoreReply* resp) {
               return service->getKPPosDescScore(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      Kp2d_method_names[3],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< Kp2d::Service, ::KP2D::ImgAndThr, ::KP2D::GetKPPosDescScoreMapReply, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](Kp2d::Service* service,
             ::grpc::ServerContext* ctx,
             const ::KP2D::ImgAndThr* req,
             ::KP2D::GetKPPosDescScoreMapReply* resp) {
               return service->getKPPosDescScoreMap(ctx, req, resp);
             }, this)));
}

Kp2d::Service::~Service() {
}

::grpc::Status Kp2d::Service::align(::grpc::ServerContext* context, const ::KP2D::ImagePair* request, ::KP2D::AlignReply* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status Kp2d::Service::getEssentialMatrix(::grpc::ServerContext* context, const ::KP2D::ImagePair* request, ::KP2D::GetEssentialMatrixReply* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status Kp2d::Service::getKPPosDescScore(::grpc::ServerContext* context, const ::KP2D::ImgAndThr* request, ::KP2D::GetKPPosDescScoreReply* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status Kp2d::Service::getKPPosDescScoreMap(::grpc::ServerContext* context, const ::KP2D::ImgAndThr* request, ::KP2D::GetKPPosDescScoreMapReply* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace KP2D

