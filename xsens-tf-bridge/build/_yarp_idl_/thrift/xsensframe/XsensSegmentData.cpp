// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <thrift/XsensSegmentData.h>

namespace xsens {
bool XsensSegmentData::read_position(yarp::os::idl::WireReader& reader) {
  if (!reader.read(position)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::nested_read_position(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(position)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::read_velocity(yarp::os::idl::WireReader& reader) {
  if (!reader.read(velocity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::nested_read_velocity(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(velocity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::read_acceleration(yarp::os::idl::WireReader& reader) {
  if (!reader.read(acceleration)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::nested_read_acceleration(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(acceleration)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::read_orientation(yarp::os::idl::WireReader& reader) {
  if (!reader.read(orientation)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::nested_read_orientation(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(orientation)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::read_angularVelocity(yarp::os::idl::WireReader& reader) {
  if (!reader.read(angularVelocity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::nested_read_angularVelocity(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(angularVelocity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::read_angularAcceleration(yarp::os::idl::WireReader& reader) {
  if (!reader.read(angularAcceleration)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::nested_read_angularAcceleration(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(angularAcceleration)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensSegmentData::read(yarp::os::idl::WireReader& reader) {
  if (!read_position(reader)) return false;
  if (!read_velocity(reader)) return false;
  if (!read_acceleration(reader)) return false;
  if (!read_orientation(reader)) return false;
  if (!read_angularVelocity(reader)) return false;
  if (!read_angularAcceleration(reader)) return false;
  return !reader.isError();
}

bool XsensSegmentData::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(19)) return false;
  return read(reader);
}

bool XsensSegmentData::write_position(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(position)) return false;
  return true;
}
bool XsensSegmentData::nested_write_position(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(position)) return false;
  return true;
}
bool XsensSegmentData::write_velocity(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(velocity)) return false;
  return true;
}
bool XsensSegmentData::nested_write_velocity(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(velocity)) return false;
  return true;
}
bool XsensSegmentData::write_acceleration(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(acceleration)) return false;
  return true;
}
bool XsensSegmentData::nested_write_acceleration(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(acceleration)) return false;
  return true;
}
bool XsensSegmentData::write_orientation(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(orientation)) return false;
  return true;
}
bool XsensSegmentData::nested_write_orientation(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(orientation)) return false;
  return true;
}
bool XsensSegmentData::write_angularVelocity(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(angularVelocity)) return false;
  return true;
}
bool XsensSegmentData::nested_write_angularVelocity(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(angularVelocity)) return false;
  return true;
}
bool XsensSegmentData::write_angularAcceleration(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(angularAcceleration)) return false;
  return true;
}
bool XsensSegmentData::nested_write_angularAcceleration(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(angularAcceleration)) return false;
  return true;
}
bool XsensSegmentData::write(yarp::os::idl::WireWriter& writer) {
  if (!write_position(writer)) return false;
  if (!write_velocity(writer)) return false;
  if (!write_acceleration(writer)) return false;
  if (!write_orientation(writer)) return false;
  if (!write_angularVelocity(writer)) return false;
  if (!write_angularAcceleration(writer)) return false;
  return !writer.isError();
}

bool XsensSegmentData::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(19)) return false;
  return write(writer);
}
bool XsensSegmentData::Editor::write(yarp::os::ConnectionWriter& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_position) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("position")) return false;
    if (!obj->nested_write_position(writer)) return false;
  }
  if (is_dirty_velocity) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("velocity")) return false;
    if (!obj->nested_write_velocity(writer)) return false;
  }
  if (is_dirty_acceleration) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("acceleration")) return false;
    if (!obj->nested_write_acceleration(writer)) return false;
  }
  if (is_dirty_orientation) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("orientation")) return false;
    if (!obj->nested_write_orientation(writer)) return false;
  }
  if (is_dirty_angularVelocity) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("angularVelocity")) return false;
    if (!obj->nested_write_angularVelocity(writer)) return false;
  }
  if (is_dirty_angularAcceleration) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("angularAcceleration")) return false;
    if (!obj->nested_write_angularAcceleration(writer)) return false;
  }
  return !writer.isError();
}
bool XsensSegmentData::Editor::read(yarp::os::ConnectionReader& connection) {
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
      if (field=="position") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector3 position")) return false;
        if (!writer.writeString("Position of the origin of the segment frame")) return false;
      }
      if (field=="velocity") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector3 velocity")) return false;
        if (!writer.writeString("linear velocity of the segment frame")) return false;
      }
      if (field=="acceleration") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector3 acceleration")) return false;
        if (!writer.writeString("linear acceleration of the segment frame")) return false;
      }
      if (field=="orientation") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector4 orientation")) return false;
        if (!writer.writeString("orientation of the segment frame in quaternion")) return false;
      }
      if (field=="angularVelocity") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector3 angularVelocity")) return false;
        if (!writer.writeString("angular velocity of the segment frame")) return false;
      }
      if (field=="angularAcceleration") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Vector3 angularAcceleration")) return false;
        if (!writer.writeString("angular acceleration of the segment frame")) return false;
      }
    }
    if (!writer.writeListHeader(7)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("position");
    writer.writeString("velocity");
    writer.writeString("acceleration");
    writer.writeString("orientation");
    writer.writeString("angularVelocity");
    writer.writeString("angularAcceleration");
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
    if (key == "position") {
      will_set_position();
      if (!obj->nested_read_position(reader)) return false;
      did_set_position();
    } else if (key == "velocity") {
      will_set_velocity();
      if (!obj->nested_read_velocity(reader)) return false;
      did_set_velocity();
    } else if (key == "acceleration") {
      will_set_acceleration();
      if (!obj->nested_read_acceleration(reader)) return false;
      did_set_acceleration();
    } else if (key == "orientation") {
      will_set_orientation();
      if (!obj->nested_read_orientation(reader)) return false;
      did_set_orientation();
    } else if (key == "angularVelocity") {
      will_set_angularVelocity();
      if (!obj->nested_read_angularVelocity(reader)) return false;
      did_set_angularVelocity();
    } else if (key == "angularAcceleration") {
      will_set_angularAcceleration();
      if (!obj->nested_read_angularAcceleration(reader)) return false;
      did_set_angularAcceleration();
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

yarp::os::ConstString XsensSegmentData::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
} // namespace
