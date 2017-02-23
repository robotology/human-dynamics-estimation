// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_XsensSegmentData
#define YARP_THRIFT_GENERATOR_STRUCT_XsensSegmentData

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <thrift/Vector3.h>
#include <thrift/Vector4.h>

namespace xsens {
  class XsensSegmentData;
}


/**
 * Data characterizing a segment.
 * It is composed of origin position, orientation in quaternion,
 * linear and angular velocity and acceleration.\
 * All the quantities are written with respect a general world frame.
 */
class xsens::XsensSegmentData : public yarp::os::idl::WirePortable {
public:
  // Fields
  /**
   * Position of the origin of the segment frame
   */
  Vector3 position;
  /**
   * linear velocity of the segment frame
   */
  Vector3 velocity;
  /**
   * linear acceleration of the segment frame
   */
  Vector3 acceleration;
  /**
   * orientation of the segment frame in quaternion
   */
  Vector4 orientation;
  /**
   * angular velocity of the segment frame
   */
  Vector3 angularVelocity;
  /**
   * angular acceleration of the segment frame
   */
  Vector3 angularAcceleration;

  // Default constructor
  XsensSegmentData() {
  }

  // Constructor with field values
  XsensSegmentData(const Vector3& position,const Vector3& velocity,const Vector3& acceleration,const Vector4& orientation,const Vector3& angularVelocity,const Vector3& angularAcceleration) : position(position), velocity(velocity), acceleration(acceleration), orientation(orientation), angularVelocity(angularVelocity), angularAcceleration(angularAcceleration) {
  }

  // Copy constructor
  XsensSegmentData(const XsensSegmentData& __alt) : WirePortable(__alt)  {
    this->position = __alt.position;
    this->velocity = __alt.velocity;
    this->acceleration = __alt.acceleration;
    this->orientation = __alt.orientation;
    this->angularVelocity = __alt.angularVelocity;
    this->angularAcceleration = __alt.angularAcceleration;
  }

  // Assignment operator
  const XsensSegmentData& operator = (const XsensSegmentData& __alt) {
    this->position = __alt.position;
    this->velocity = __alt.velocity;
    this->acceleration = __alt.acceleration;
    this->orientation = __alt.orientation;
    this->angularVelocity = __alt.angularVelocity;
    this->angularAcceleration = __alt.angularAcceleration;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);

private:
  bool write_position(yarp::os::idl::WireWriter& writer);
  bool nested_write_position(yarp::os::idl::WireWriter& writer);
  bool write_velocity(yarp::os::idl::WireWriter& writer);
  bool nested_write_velocity(yarp::os::idl::WireWriter& writer);
  bool write_acceleration(yarp::os::idl::WireWriter& writer);
  bool nested_write_acceleration(yarp::os::idl::WireWriter& writer);
  bool write_orientation(yarp::os::idl::WireWriter& writer);
  bool nested_write_orientation(yarp::os::idl::WireWriter& writer);
  bool write_angularVelocity(yarp::os::idl::WireWriter& writer);
  bool nested_write_angularVelocity(yarp::os::idl::WireWriter& writer);
  bool write_angularAcceleration(yarp::os::idl::WireWriter& writer);
  bool nested_write_angularAcceleration(yarp::os::idl::WireWriter& writer);
  bool read_position(yarp::os::idl::WireReader& reader);
  bool nested_read_position(yarp::os::idl::WireReader& reader);
  bool read_velocity(yarp::os::idl::WireReader& reader);
  bool nested_read_velocity(yarp::os::idl::WireReader& reader);
  bool read_acceleration(yarp::os::idl::WireReader& reader);
  bool nested_read_acceleration(yarp::os::idl::WireReader& reader);
  bool read_orientation(yarp::os::idl::WireReader& reader);
  bool nested_read_orientation(yarp::os::idl::WireReader& reader);
  bool read_angularVelocity(yarp::os::idl::WireReader& reader);
  bool nested_read_angularVelocity(yarp::os::idl::WireReader& reader);
  bool read_angularAcceleration(yarp::os::idl::WireReader& reader);
  bool nested_read_angularAcceleration(yarp::os::idl::WireReader& reader);

public:

  yarp::os::ConstString toString();

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<xsens::XsensSegmentData > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new XsensSegmentData;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(XsensSegmentData& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(XsensSegmentData& obj, bool dirty = true) {
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

    XsensSegmentData& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_position(const Vector3& position) {
      will_set_position();
      obj->position = position;
      mark_dirty_position();
      communicate();
      did_set_position();
    }
    void set_velocity(const Vector3& velocity) {
      will_set_velocity();
      obj->velocity = velocity;
      mark_dirty_velocity();
      communicate();
      did_set_velocity();
    }
    void set_acceleration(const Vector3& acceleration) {
      will_set_acceleration();
      obj->acceleration = acceleration;
      mark_dirty_acceleration();
      communicate();
      did_set_acceleration();
    }
    void set_orientation(const Vector4& orientation) {
      will_set_orientation();
      obj->orientation = orientation;
      mark_dirty_orientation();
      communicate();
      did_set_orientation();
    }
    void set_angularVelocity(const Vector3& angularVelocity) {
      will_set_angularVelocity();
      obj->angularVelocity = angularVelocity;
      mark_dirty_angularVelocity();
      communicate();
      did_set_angularVelocity();
    }
    void set_angularAcceleration(const Vector3& angularAcceleration) {
      will_set_angularAcceleration();
      obj->angularAcceleration = angularAcceleration;
      mark_dirty_angularAcceleration();
      communicate();
      did_set_angularAcceleration();
    }
    const Vector3& get_position() {
      return obj->position;
    }
    const Vector3& get_velocity() {
      return obj->velocity;
    }
    const Vector3& get_acceleration() {
      return obj->acceleration;
    }
    const Vector4& get_orientation() {
      return obj->orientation;
    }
    const Vector3& get_angularVelocity() {
      return obj->angularVelocity;
    }
    const Vector3& get_angularAcceleration() {
      return obj->angularAcceleration;
    }
    virtual bool will_set_position() { return true; }
    virtual bool will_set_velocity() { return true; }
    virtual bool will_set_acceleration() { return true; }
    virtual bool will_set_orientation() { return true; }
    virtual bool will_set_angularVelocity() { return true; }
    virtual bool will_set_angularAcceleration() { return true; }
    virtual bool did_set_position() { return true; }
    virtual bool did_set_velocity() { return true; }
    virtual bool did_set_acceleration() { return true; }
    virtual bool did_set_orientation() { return true; }
    virtual bool did_set_angularVelocity() { return true; }
    virtual bool did_set_angularAcceleration() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:

    XsensSegmentData *obj;

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
    void mark_dirty_position() {
      if (is_dirty_position) return;
      dirty_count++;
      is_dirty_position = true;
      mark_dirty();
    }
    void mark_dirty_velocity() {
      if (is_dirty_velocity) return;
      dirty_count++;
      is_dirty_velocity = true;
      mark_dirty();
    }
    void mark_dirty_acceleration() {
      if (is_dirty_acceleration) return;
      dirty_count++;
      is_dirty_acceleration = true;
      mark_dirty();
    }
    void mark_dirty_orientation() {
      if (is_dirty_orientation) return;
      dirty_count++;
      is_dirty_orientation = true;
      mark_dirty();
    }
    void mark_dirty_angularVelocity() {
      if (is_dirty_angularVelocity) return;
      dirty_count++;
      is_dirty_angularVelocity = true;
      mark_dirty();
    }
    void mark_dirty_angularAcceleration() {
      if (is_dirty_angularAcceleration) return;
      dirty_count++;
      is_dirty_angularAcceleration = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_position = flag;
      is_dirty_velocity = flag;
      is_dirty_acceleration = flag;
      is_dirty_orientation = flag;
      is_dirty_angularVelocity = flag;
      is_dirty_angularAcceleration = flag;
      dirty_count = flag ? 6 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_position;
    bool is_dirty_velocity;
    bool is_dirty_acceleration;
    bool is_dirty_orientation;
    bool is_dirty_angularVelocity;
    bool is_dirty_angularAcceleration;
  };
};

#endif
