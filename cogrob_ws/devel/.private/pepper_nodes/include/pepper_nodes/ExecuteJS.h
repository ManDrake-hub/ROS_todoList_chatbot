// Generated by gencpp from file pepper_nodes/ExecuteJS.msg
// DO NOT EDIT!


#ifndef PEPPER_NODES_MESSAGE_EXECUTEJS_H
#define PEPPER_NODES_MESSAGE_EXECUTEJS_H

#include <ros/service_traits.h>


#include <pepper_nodes/ExecuteJSRequest.h>
#include <pepper_nodes/ExecuteJSResponse.h>


namespace pepper_nodes
{

struct ExecuteJS
{

typedef ExecuteJSRequest Request;
typedef ExecuteJSResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ExecuteJS
} // namespace pepper_nodes


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::pepper_nodes::ExecuteJS > {
  static const char* value()
  {
    return "0bc1212ef5c5830fe8dbd8060c89a89d";
  }

  static const char* value(const ::pepper_nodes::ExecuteJS&) { return value(); }
};

template<>
struct DataType< ::pepper_nodes::ExecuteJS > {
  static const char* value()
  {
    return "pepper_nodes/ExecuteJS";
  }

  static const char* value(const ::pepper_nodes::ExecuteJS&) { return value(); }
};


// service_traits::MD5Sum< ::pepper_nodes::ExecuteJSRequest> should match
// service_traits::MD5Sum< ::pepper_nodes::ExecuteJS >
template<>
struct MD5Sum< ::pepper_nodes::ExecuteJSRequest>
{
  static const char* value()
  {
    return MD5Sum< ::pepper_nodes::ExecuteJS >::value();
  }
  static const char* value(const ::pepper_nodes::ExecuteJSRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::pepper_nodes::ExecuteJSRequest> should match
// service_traits::DataType< ::pepper_nodes::ExecuteJS >
template<>
struct DataType< ::pepper_nodes::ExecuteJSRequest>
{
  static const char* value()
  {
    return DataType< ::pepper_nodes::ExecuteJS >::value();
  }
  static const char* value(const ::pepper_nodes::ExecuteJSRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::pepper_nodes::ExecuteJSResponse> should match
// service_traits::MD5Sum< ::pepper_nodes::ExecuteJS >
template<>
struct MD5Sum< ::pepper_nodes::ExecuteJSResponse>
{
  static const char* value()
  {
    return MD5Sum< ::pepper_nodes::ExecuteJS >::value();
  }
  static const char* value(const ::pepper_nodes::ExecuteJSResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::pepper_nodes::ExecuteJSResponse> should match
// service_traits::DataType< ::pepper_nodes::ExecuteJS >
template<>
struct DataType< ::pepper_nodes::ExecuteJSResponse>
{
  static const char* value()
  {
    return DataType< ::pepper_nodes::ExecuteJS >::value();
  }
  static const char* value(const ::pepper_nodes::ExecuteJSResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PEPPER_NODES_MESSAGE_EXECUTEJS_H