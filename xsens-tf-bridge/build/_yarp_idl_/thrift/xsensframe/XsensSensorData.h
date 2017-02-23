// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_XsensSensorData
#define YARP_THRIFT_GENERATOR_STRUCT_XsensSensorData

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <thrift/Vector3.h>
#include <thrift/Vector4.h>

namespace xsens {
  class XsensSensorData;
}


/**
 * Data characterizing sensor data.
 * It is composed of orientation in quaternion,
 * angular velocity acceleration and magnetometer information
 */
class xsens::XsensSensorData : public yarp::os::idl::WirePortable {
public:
  // Fields
  /**
   * sensor acceleration in m / s^2
   */
  Vector3 acceleration;
  /**
   * sensor angular velocity in deg / s
   */
  Vector3 angularVelocity;
  /**
   * The magnetometer data in arbitrary units
   */
  Vector3 magnetometer;
  /**
   * The sensor computed orientation
   */
  Vector4 orientation;

  // Default constructor
  XsensSensorData() {
  }

  // Constructor with field values
  XsensSensorData(const Vector3& acceleration,const Vector3& angularVelocity,const Vector3& magnetometer,const Vector4& orientation) : acceleration(acceleration), angularVelocity(angularVelocity), magnetometer(magnetometer), orientation(orientation) {
  }

  // Copy constructor
  XsensSensorData(const XsensSensorData& __alt) : WirePortable(__alt)  {
    this->acceleration = __alt.acceleration;
    this->angularVelocity = __alt.angularVelocity;
    this->magnetometer = __alt.magnetometer;
    this->orientation = __alt.orientation;
  }

  // Assignment operator
  const XsensSensorData& operator = (const XsensSensorData& __alt) {
    this->acceleration = __alt.acceleration;
    this->angularVelocity = __alt.angularVelocity;
    this->magnetometer = __alt.magnetometer;
    this->orientation = __alt.orientation;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);

private:
  bool write_acceleration(yarp::os::idl::WireWriter& writer);
  bool nested_write_acceleration(yarp::os::idl::WireWriter& writer);
  bool write_angularVelocity(yarp::os::idl::WireWriter& writer);
  bool nested_write_angularVelocity(yarp::os::idl::WireWriter& writer);
  bool write_magnetometer(yarp::os::idl::WireWriter& writer);
  bool nested_write_magnetometer(yarp::os::idl::WireWriter& writer);
  bool write_orientation(yarp::os::idl::WireWriter& writer);
  bool nested_write_orientation(yarp::os::idl::WireWriter& writer);
  bool read_acceleration(yarp::os::idl::WireReader& reader);
  bool nested_read_acceleration(yarp::os::idl::WireReader& reader);
  bool read_angularVelocity(yarp::os::idl::WireReader& reader);
  bool nested_read_angularVelocity(yarp::os::idl::WireReader& reader);
  bool read_magnetometer(yarp::os::idl::WireReader& reader);
  bool nested_read_magnetometer(yarp::os::idl::WireReader& reader);
  bool read_orientation(yarp::os::idl::WireReader& reader);
  bool nested_read_orientation(yarp::os::idl::WireReader& reader);

public:

  yarp::os::ConstString toString();

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<xsens::XsensSensorData > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new XsensSensorData;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(XsensSensorData& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(XsensSensorData& obj, bool dirty = true) {
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

    XsensSensorData& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_acceleration(const Vector3& acceleration) {
      will_set_acceleration();
      obj->acceleration = acceleration;
      mark_dirty_acceleration();
      communicate();
      did_set_acceleration();
    }
    void set_angularVelocity(const Vector3& angularVelocity) {
      will_set_angularVelocity();
      obj->angularVelocity = angularVelocity;
      mark_dirty_angularVelocity();
      communicate();
      did_set_angularVelocity();
    }
    void set_magnetometer(const Vector3& magnetometer) {
      will_set_magnetometer();
      obj->magnetometer = magnetometer;
      mark_dirty_magnetometer();
      communicate();
      did_set_magnetometer();
    }
    void set_orientation(const Vector4& orientation) {
      will_set_orientation();
      obj->orientation = orientation;
      mark_dirty_orientation();
      communicate();
      did_set_orientation();
    }
    const Vector3& get_acceleration() {
      return obj->acceleration;
    }
    const Vector3& get_angularVelocity() {
      return obj->angularVelocity;
    }
    const Vector3& get_magnetometer() {
      return obj->magnetometer;
    }
    const Vector4& get_orientation() {
      return obj->orientation;
    }
    virtual bool will_set_acceleration() { return true; }
    virtual bool will_set_angularVelocity() { return true; }
    virtual bool will_set_magnetometer() { return true; }
    virtual bool will_set_orientation() { return true; }
    virtual bool did_set_acceleration() { return true; }
    virtual bool did_set_angularVelocity() { return true; }
    virtual bool did_set_magnetometer() { return true; }
    virtual bool did_set_orientation() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:

    XsensSensorData *obj;

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
    void mark_dirty_acceleration() {
      if (is_dirty_acceleration) return;
      dirty_count++;
      is_dirty_acceleration = true;
      mark_dirty();
    }
    void mark_dirty_angularVelocity() {
      if (is_dirty_angularVelocity) return;
      dirty_count++;
      is_dirty_angularVelocity = true;
      mark_dirty();
    }
    void mark_dirty_magnetometer() {
      if (is_dirty_magnetometer) return;
      dirty_count++;
      is_dirty_magnetometer = true;
      mark_dirty();
    }
    void mark_dirty_orientation() {
      if (is_dirty_orientation) return;
      dirty_count++;
      is_dirty_orientation = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_acceleration = flag;
      is_dirty_angularVelocity = flag;
      is_dirty_magnetometer = flag;
      is_dirty_orientation = flag;
      dirty_count = flag ? 4 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_acceleration;
    bool is_dirty_angularVelocity;
    bool is_dirty_magnetometer;
    bool is_dirty_orientation;
  };
};

#endif
