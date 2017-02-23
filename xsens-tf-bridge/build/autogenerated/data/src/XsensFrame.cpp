// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <thrift/XsensFrame.h>

namespace xsens {
bool XsensFrame::read_time(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(time)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensFrame::nested_read_time(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(time)) {
    reader.fail();
    return false;
  }
  return true;
}
bool XsensFrame::read_segmentsData(yarp::os::idl::WireReader& reader) {
  {
    segmentsData.clear();
    uint32_t _size0;
    yarp::os::idl::WireState _etype3;
    reader.readListBegin(_etype3, _size0);
    segmentsData.resize(_size0);
    uint32_t _i4;
    for (_i4 = 0; _i4 < _size0; ++_i4)
    {
      if (!reader.readNested(segmentsData[_i4])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return true;
}
bool XsensFrame::nested_read_segmentsData(yarp::os::idl::WireReader& reader) {
  {
    segmentsData.clear();
    uint32_t _size5;
    yarp::os::idl::WireState _etype8;
    reader.readListBegin(_etype8, _size5);
    segmentsData.resize(_size5);
    uint32_t _i9;
    for (_i9 = 0; _i9 < _size5; ++_i9)
    {
      if (!reader.readNested(segmentsData[_i9])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return true;
}
bool XsensFrame::read_sensorsData(yarp::os::idl::WireReader& reader) {
  {
    sensorsData.clear();
    uint32_t _size10;
    yarp::os::idl::WireState _etype13;
    reader.readListBegin(_etype13, _size10);
    sensorsData.resize(_size10);
    uint32_t _i14;
    for (_i14 = 0; _i14 < _size10; ++_i14)
    {
      if (!reader.readNested(sensorsData[_i14])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return true;
}
bool XsensFrame::nested_read_sensorsData(yarp::os::idl::WireReader& reader) {
  {
    sensorsData.clear();
    uint32_t _size15;
    yarp::os::idl::WireState _etype18;
    reader.readListBegin(_etype18, _size15);
    sensorsData.resize(_size15);
    uint32_t _i19;
    for (_i19 = 0; _i19 < _size15; ++_i19)
    {
      if (!reader.readNested(sensorsData[_i19])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return true;
}
bool XsensFrame::read(yarp::os::idl::WireReader& reader) {
  if (!read_time(reader)) return false;
  if (!read_segmentsData(reader)) return false;
  if (!read_sensorsData(reader)) return false;
  return !reader.isError();
}

bool XsensFrame::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(3)) return false;
  return read(reader);
}

bool XsensFrame::write_time(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(time)) return false;
  return true;
}
bool XsensFrame::nested_write_time(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(time)) return false;
  return true;
}
bool XsensFrame::write_segmentsData(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(segmentsData.size()))) return false;
    std::vector<XsensSegmentData> ::iterator _iter20;
    for (_iter20 = segmentsData.begin(); _iter20 != segmentsData.end(); ++_iter20)
    {
      if (!writer.writeNested((*_iter20))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}
bool XsensFrame::nested_write_segmentsData(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(segmentsData.size()))) return false;
    std::vector<XsensSegmentData> ::iterator _iter21;
    for (_iter21 = segmentsData.begin(); _iter21 != segmentsData.end(); ++_iter21)
    {
      if (!writer.writeNested((*_iter21))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}
bool XsensFrame::write_sensorsData(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(sensorsData.size()))) return false;
    std::vector<XsensSensorData> ::iterator _iter22;
    for (_iter22 = sensorsData.begin(); _iter22 != sensorsData.end(); ++_iter22)
    {
      if (!writer.writeNested((*_iter22))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}
bool XsensFrame::nested_write_sensorsData(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(sensorsData.size()))) return false;
    std::vector<XsensSensorData> ::iterator _iter23;
    for (_iter23 = sensorsData.begin(); _iter23 != sensorsData.end(); ++_iter23)
    {
      if (!writer.writeNested((*_iter23))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}
bool XsensFrame::write(yarp::os::idl::WireWriter& writer) {
  if (!write_time(writer)) return false;
  if (!write_segmentsData(writer)) return false;
  if (!write_sensorsData(writer)) return false;
  return !writer.isError();
}

bool XsensFrame::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  return write(writer);
}
bool XsensFrame::Editor::write(yarp::os::ConnectionWriter& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_time) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("time")) return false;
    if (!obj->nested_write_time(writer)) return false;
  }
  if (is_dirty_segmentsData) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("segmentsData")) return false;
    if (!obj->nested_write_segmentsData(writer)) return false;
  }
  if (is_dirty_sensorsData) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("sensorsData")) return false;
    if (!obj->nested_write_sensorsData(writer)) return false;
  }
  return !writer.isError();
}
bool XsensFrame::Editor::read(yarp::os::ConnectionReader& connection) {
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
      if (field=="time") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double time")) return false;
        if (!writer.writeString("absolute time in seconds")) return false;
      }
      if (field=="segmentsData") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("std::vector<XsensSegmentData>  segmentsData")) return false;
        if (!writer.writeString("segments data")) return false;
      }
      if (field=="sensorsData") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("std::vector<XsensSensorData>  sensorsData")) return false;
        if (!writer.writeString("sensors data")) return false;
      }
    }
    if (!writer.writeListHeader(4)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("time");
    writer.writeString("segmentsData");
    writer.writeString("sensorsData");
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
    if (key == "time") {
      will_set_time();
      if (!obj->nested_read_time(reader)) return false;
      did_set_time();
    } else if (key == "segmentsData") {
      will_set_segmentsData();
      if (!obj->nested_read_segmentsData(reader)) return false;
      did_set_segmentsData();
    } else if (key == "sensorsData") {
      will_set_sensorsData();
      if (!obj->nested_read_sensorsData(reader)) return false;
      did_set_sensorsData();
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

yarp::os::ConstString XsensFrame::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
} // namespace
