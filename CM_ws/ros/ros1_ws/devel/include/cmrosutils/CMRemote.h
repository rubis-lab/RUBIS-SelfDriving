// Generated by gencpp from file cmrosutils/CMRemote.msg
// DO NOT EDIT!


#ifndef CMROSUTILS_MESSAGE_CMREMOTE_H
#define CMROSUTILS_MESSAGE_CMREMOTE_H

#include <ros/service_traits.h>


#include <cmrosutils/CMRemoteRequest.h>
#include <cmrosutils/CMRemoteResponse.h>


namespace cmrosutils
{

struct CMRemote
{

typedef CMRemoteRequest Request;
typedef CMRemoteResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CMRemote
} // namespace cmrosutils


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cmrosutils::CMRemote > {
  static const char* value()
  {
    return "1d9ce7b684aec19fba170323f6c05b62";
  }

  static const char* value(const ::cmrosutils::CMRemote&) { return value(); }
};

template<>
struct DataType< ::cmrosutils::CMRemote > {
  static const char* value()
  {
    return "cmrosutils/CMRemote";
  }

  static const char* value(const ::cmrosutils::CMRemote&) { return value(); }
};


// service_traits::MD5Sum< ::cmrosutils::CMRemoteRequest> should match
// service_traits::MD5Sum< ::cmrosutils::CMRemote >
template<>
struct MD5Sum< ::cmrosutils::CMRemoteRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cmrosutils::CMRemote >::value();
  }
  static const char* value(const ::cmrosutils::CMRemoteRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cmrosutils::CMRemoteRequest> should match
// service_traits::DataType< ::cmrosutils::CMRemote >
template<>
struct DataType< ::cmrosutils::CMRemoteRequest>
{
  static const char* value()
  {
    return DataType< ::cmrosutils::CMRemote >::value();
  }
  static const char* value(const ::cmrosutils::CMRemoteRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cmrosutils::CMRemoteResponse> should match
// service_traits::MD5Sum< ::cmrosutils::CMRemote >
template<>
struct MD5Sum< ::cmrosutils::CMRemoteResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cmrosutils::CMRemote >::value();
  }
  static const char* value(const ::cmrosutils::CMRemoteResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cmrosutils::CMRemoteResponse> should match
// service_traits::DataType< ::cmrosutils::CMRemote >
template<>
struct DataType< ::cmrosutils::CMRemoteResponse>
{
  static const char* value()
  {
    return DataType< ::cmrosutils::CMRemote >::value();
  }
  static const char* value(const ::cmrosutils::CMRemoteResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CMROSUTILS_MESSAGE_CMREMOTE_H