// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_XsensFrame
#define YARP_THRIFT_GENERATOR_STRUCT_XsensFrame

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <thrift/XsensSegmentData.h>
#include <thrift/XsensSensorData.h>

namespace xsens {
  class XsensFrame;
}


/**
 * Frame output from Xsens
 */
class xsens::XsensFrame : public yarp::os::idl::WirePortable {
public:
  // Fields
  /**
   * absolute time in seconds
   */
  double time;
  /**
   * segments data
   */
  std::vector<XsensSegmentData>  segmentsData;
  /**
   * sensors data
   */
  std::vector<XsensSensorData>  sensorsData;

  // Default constructor
  XsensFrame() : time(0) {
  }

  // Constructor with field values
  XsensFrame(const double time,const std::vector<XsensSegmentData> & segmentsData,const std::vector<XsensSensorData> & sensorsData) : time(time), segmentsData(segmentsData), sensorsData(sensorsData) {
  }

  // Copy constructor
  XsensFrame(const XsensFrame& __alt) : WirePortable(__alt)  {
    this->time = __alt.time;
    this->segmentsData = __alt.segmentsData;
    this->sensorsData = __alt.sensorsData;
  }

  // Assignment operator
  const XsensFrame& operator = (const XsensFrame& __alt) {
    this->time = __alt.time;
    this->segmentsData = __alt.segmentsData;
    this->sensorsData = __alt.sensorsData;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);

private:
  bool write_time(yarp::os::idl::WireWriter& writer);
  bool nested_write_time(yarp::os::idl::WireWriter& writer);
  bool write_segmentsData(yarp::os::idl::WireWriter& writer);
  bool nested_write_segmentsData(yarp::os::idl::WireWriter& writer);
  bool write_sensorsData(yarp::os::idl::WireWriter& writer);
  bool nested_write_sensorsData(yarp::os::idl::WireWriter& writer);
  bool read_time(yarp::os::idl::WireReader& reader);
  bool nested_read_time(yarp::os::idl::WireReader& reader);
  bool read_segmentsData(yarp::os::idl::WireReader& reader);
  bool nested_read_segmentsData(yarp::os::idl::WireReader& reader);
  bool read_sensorsData(yarp::os::idl::WireReader& reader);
  bool nested_read_sensorsData(yarp::os::idl::WireReader& reader);

public:

  yarp::os::ConstString toString();

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<xsens::XsensFrame > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new XsensFrame;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(XsensFrame& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(XsensFrame& obj, bool dirty = true) {
      if (obj_owned) delete this->obj;
      this->obj = &obj;
      obj_owned = false;
      dirty_flags(dirty);
      return true;
    }

    virtual ~Editor() {
    if (obj_owned) delete obj;
    }

    bool isValid() const {
      return obj!=0/*NULL*/;
    }

    XsensFrame& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_time(const double time) {
      will_set_time();
      obj->time = time;
      mark_dirty_time();
      communicate();
      did_set_time();
    }
    void set_segmentsData(const std::vector<XsensSegmentData> & segmentsData) {
      will_set_segmentsData();
      obj->segmentsData = segmentsData;
      mark_dirty_segmentsData();
      communicate();
      did_set_segmentsData();
    }
    void set_segmentsData(int index, const XsensSegmentData& elem) {
      will_set_segmentsData();
      obj->segmentsData[index] = elem;
      mark_dirty_segmentsData();
      communicate();
      did_set_segmentsData();
    }
    void set_sensorsData(const std::vector<XsensSensorData> & sensorsData) {
      will_set_sensorsData();
      obj->sensorsData = sensorsData;
      mark_dirty_sensorsData();
      communicate();
      did_set_sensorsData();
    }
    void set_sensorsData(int index, const XsensSensorData& elem) {
      will_set_sensorsData();
      obj->sensorsData[index] = elem;
      mark_dirty_sensorsData();
      communicate();
      did_set_sensorsData();
    }
    double get_time() {
      return obj->time;
    }
    const std::vector<XsensSegmentData> & get_segmentsData() {
      return obj->segmentsData;
    }
    const std::vector<XsensSensorData> & get_sensorsData() {
      return obj->sensorsData;
    }
    virtual bool will_set_time() { return true; }
    virtual bool will_set_segmentsData() { return true; }
    virtual bool will_set_sensorsData() { return true; }
    virtual bool did_set_time() { return true; }
    virtual bool did_set_segmentsData() { return true; }
    virtual bool did_set_sensorsData() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:

    XsensFrame *obj;

    bool obj_owned;
    int group;

    void communicate() {
      if (group!=0) return;
      if (yarp().canWrite()) {
        yarp().write(*this);
        clean();
      }
    }
    void mark_dirty() {
      is_dirty = true;
    }
    void mark_dirty_time() {
      if (is_dirty_time) return;
      dirty_count++;
      is_dirty_time = true;
      mark_dirty();
    }
    void mark_dirty_segmentsData() {
      if (is_dirty_segmentsData) return;
      dirty_count++;
      is_dirty_segmentsData = true;
      mark_dirty();
    }
    void mark_dirty_sensorsData() {
      if (is_dirty_sensorsData) return;
      dirty_count++;
      is_dirty_sensorsData = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_time = flag;
      is_dirty_segmentsData = flag;
      is_dirty_sensorsData = flag;
      dirty_count = flag ? 3 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_time;
    bool is_dirty_segmentsData;
    bool is_dirty_sensorsData;
  };
};

#endif
