// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <thrift/Vector4.h>

namespace xsens {
bool Vector4::read_c1(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(c1)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Vector4::nested_read_c1(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(c1)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Vector4::read_c2(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(c2)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Vector4::nested_read_c2(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(c2)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Vector4::read_c3(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(c3)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Vector4::nested_read_c3(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(c3)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Vector4::read_c4(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(c4)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Vector4::nested_read_c4(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(c4)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Vector4::read(yarp::os::idl::WireReader& reader) {
  if (!read_c1(reader)) return false;
  if (!read_c2(reader)) return false;
  if (!read_c3(reader)) return false;
  if (!read_c4(reader)) return false;
  return !reader.isError();
}

bool Vector4::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(4)) return false;
  return read(reader);
}

bool Vector4::write_c1(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(c1)) return false;
  return true;
}
bool Vector4::nested_write_c1(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(c1)) return false;
  return true;
}
bool Vector4::write_c2(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(c2)) return false;
  return true;
}
bool Vector4::nested_write_c2(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(c2)) return false;
  return true;
}
bool Vector4::write_c3(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(c3)) return false;
  return true;
}
bool Vector4::nested_write_c3(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(c3)) return false;
  return true;
}
bool Vector4::write_c4(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(c4)) return false;
  return true;
}
bool Vector4::nested_write_c4(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(c4)) return false;
  return true;
}
bool Vector4::write(yarp::os::idl::WireWriter& writer) {
  if (!write_c1(writer)) return false;
  if (!write_c2(writer)) return false;
  if (!write_c3(writer)) return false;
  if (!write_c4(writer)) return false;
  return !writer.isError();
}

bool Vector4::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  return write(writer);
}
bool Vector4::Editor::write(yarp::os::ConnectionWriter& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_c1) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("c1")) return false;
    if (!obj->nested_write_c1(writer)) return false;
  }
  if (is_dirty_c2) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("c2")) return false;
    if (!obj->nested_write_c2(writer)) return false;
  }
  if (is_dirty_c3) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("c3")) return false;
    if (!obj->nested_write_c3(writer)) return false;
  }
  if (is_dirty_c4) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("c4")) return false;
    if (!obj->nested_write_c4(writer)) return false;
  }
  return !writer.isError();
}
bool Vector4::Editor::read(yarp::os::ConnectionReader& connection) {
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
      if (field=="c1") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double c1")) return false;
      }
      if (field=="c2") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double c2")) return false;
      }
      if (field=="c3") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double c3")) return false;
      }
      if (field=="c4") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double c4")) return false;
      }
    }
    if (!writer.writeListHeader(5)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("c1");
    writer.writeString("c2");
    writer.writeString("c3");
    writer.writeString("c4");
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
    if (key == "c1") {
      will_set_c1();
      if (!obj->nested_read_c1(reader)) return false;
      did_set_c1();
    } else if (key == "c2") {
      will_set_c2();
      if (!obj->nested_read_c2(reader)) return false;
      did_set_c2();
    } else if (key == "c3") {
      will_set_c3();
      if (!obj->nested_read_c3(reader)) return false;
      did_set_c3();
    } else if (key == "c4") {
      will_set_c4();
      if (!obj->nested_read_c4(reader)) return false;
      did_set_c4();
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

yarp::os::ConstString Vector4::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
} // namespace
