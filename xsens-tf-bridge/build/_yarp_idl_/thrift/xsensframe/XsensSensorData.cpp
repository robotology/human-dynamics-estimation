// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <thrift/XsensSensorData.h>

namespace xsens {
bool XsensSensorData::read_acceleration(yarp::os::idl::WireReader& reader) {
  if (!reader.read(acceleration)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSensorData::nested_read_acceleration(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(acceleration)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSensorData::read_angularVelocity(yarp::os::idl::WireReader& reader) {
  if (!reader.read(angularVelocity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSensorData::nested_read_angularVelocity(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(angularVelocity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSensorData::read_magnetometer(yarp::os::idl::WireReader& reader) {
  if (!reader.read(magnetometer)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSensorData::nested_read_magnetometer(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(magnetometer)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSensorData::read_orientation(yarp::os::idl::WireReader& reader) {
  if (!reader.read(orientation)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSensorData::nested_read_orientation(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(orientation)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSensorData::read(yarp::os::idl::WireReader& reader) {
  if (!read_acceleration(reader)) return false;
  if (!read_angularVelocity(reader)) return false;
  if (!read_magnetometer(reader)) return false;
  if (!read_orientation(reader)) return false;
  return !reader.isError();
}

bool XsensSensorData::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(13)) return false;
  return read(reader);
}

bool XsensSensorData::write_acceleration(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(acceleration)) return false;
  return true;
}
bool XsensSensorData::nested_write_acceleration(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(acceleration)) return false;
  return true;
}
bool XsensSensorData::write_angularVelocity(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(angularVelocity)) return false;
  return true;
}
bool XsensSensorData::nested_write_angularVelocity(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(angularVelocity)) return false;
  return true;
}
bool XsensSensorData::write_magnetometer(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(magnetometer)) return false;
  return true;
}
bool XsensSensorData::nested_write_magnetometer(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(magnetometer)) return false;
  return true;
}
bool XsensSensorData::write_orientation(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(orientation)) return false;
  return true;
}
bool XsensSensorData::nested_write_orientation(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(orientation)) return false;
  return true;
}
bool XsensSensorData::write(yarp::os::idl::WireWriter& writer) {
  if (!write_acceleration(writer)) return false;
  if (!write_angularVelocity(writer)) return false;
  if (!write_magnetometer(writer)) return false;
  if (!write_orientation(writer)) return false;
  return !writer.isError();
}

bool XsensSensorData::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(13)) return false;
  return write(writer);
}
bool XsensSensorData::Editor::write(yarp::os::ConnectionWriter& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_acceleration) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("acceleration")) return false;
    if (!obj->nested_write_acceleration(writer)) return false;
  }
  if (is_dirty_angularVelocity) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("angularVelocity")) return false;
    if (!obj->nested_write_angularVelocity(writer)) return false;
  }
  if (is_dirty_magnetometer) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("magnetometer")) return false;
    if (!obj->nested_write_magnetometer(writer)) return false;
  }
  if (is_dirty_orientation) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("orientation")) return false;
    if (!obj->nested_write_orientation(writer)) return false;
  }
  return !writer.isError();
}
bool XsensSensorData::Editor::read(yarp::os::ConnectionReader& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) return false;
  int len = reader.getLength();
  if (len==0) {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(1)) return false;
    writer.writeString("send: 'help' or 'patch (param1 val1) (param2 val2)'");
    return true;
  }
  yarp::os::ConstString tag;
  if (!reader.readString(tag)) return false;
  if (tag=="help") {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("many",1, 0)) return false;
    if (reader.getLength()>0) {
      yarp::os::ConstString field;
      if (!reader.readString(field)) return false;
      if (field=="acceleration") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector3 acceleration")) return false;
        if (!writer.writeString("sensor acceleration in m / s^2")) return false;
      }
      if (field=="angularVelocity") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector3 angularVelocity")) return false;
        if (!writer.writeString("sensor angular velocity in deg / s")) return false;
      }
      if (field=="magnetometer") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector3 magnetometer")) return false;
        if (!writer.writeString("The magnetometer data in arbitrary units")) return false;
      }
      if (field=="orientation") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector4 orientation")) return false;
        if (!writer.writeString("The sensor computed orientation")) return false;
      }
    }
    if (!writer.writeListHeader(5)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("acceleration");
    writer.writeString("angularVelocity");
    writer.writeString("magnetometer");
    writer.writeString("orientation");
    return true;
  }
  bool nested = true;
  bool have_act = false;
  if (tag!="patch") {
    if ((len-1)%2 != 0) return false;
    len = 1 + ((len-1)/2);
    nested = false;
    have_act = true;
  }
  for (int i=1; i<len; i++) {
    if (nested && !reader.readListHeader(3)) return false;
    yarp::os::ConstString act;
    yarp::os::ConstString key;
    if (have_act) {
      act = tag;
    } else {
      if (!reader.readString(act)) return false;
    }
    if (!reader.readString(key)) return false;
    // inefficient code follows, bug paulfitz to improve it
    if (key == "acceleration") {
      will_set_acceleration();
      if (!obj->nested_read_acceleration(reader)) return false;
      did_set_acceleration();
    } else if (key == "angularVelocity") {
      will_set_angularVelocity();
      if (!obj->nested_read_angularVelocity(reader)) return false;
      did_set_angularVelocity();
    } else if (key == "magnetometer") {
      will_set_magnetometer();
      if (!obj->nested_read_magnetometer(reader)) return false;
      did_set_magnetometer();
    } else if (key == "orientation") {
      will_set_orientation();
      if (!obj->nested_read_orientation(reader)) return false;
      did_set_orientation();
    } else {
      // would be useful to have a fallback here
    }
  }
  reader.accept();
  yarp::os::idl::WireWriter writer(reader);
  if (writer.isNull()) return true;
  writer.writeListHeader(1);
  writer.writeVocab(VOCAB2('o','k'));
  return true;
}

yarp::os::ConstString XsensSensorData::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
} // namespace
