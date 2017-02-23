// This is an automatically generated file.
// Generated from this String.msg definition:
//   [std_msgs/String]:
//   string data
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_String
#define YARPMSG_TYPE_String

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class String : public yarp::os::idl::WirePortable {
public:
  std::string data;

  String() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** data ***
    int len = connection.expectInt();
    data.resize(len);
    if (!connection.expectBlock((char*)data.c_str(),len)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(1)) return false;

    // *** data ***
    if (!reader.readString(data)) return false;
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** data ***
    connection.appendInt(data.length());
    connection.appendExternalBlock((char*)data.c_str(),data.length());
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(1);

    // *** data ***
    connection.appendInt(BOTTLE_TAG_STRING);
    connection.appendInt(data.length());
    connection.appendExternalBlock((char*)data.c_str(),data.length());
    connection.convertTextMode();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::write;
  bool write(yarp::os::ConnectionWriter& connection) {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<String> rosStyle;
  typedef yarp::os::idl::BottleStyle<String> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "[std_msgs/String]:\n\
string data\n\
";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("String","String");
    typ.addProperty("md5sum",yarp::os::Value("992ce8a1687cec8c8bd883ec73ca41d1"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
