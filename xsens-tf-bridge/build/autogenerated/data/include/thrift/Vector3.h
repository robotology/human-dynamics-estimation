// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_Vector3
#define YARP_THRIFT_GENERATOR_STRUCT_Vector3

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace xsens {
  class Vector3;
}


/**
 * Representation of a 3D vector
 */
class xsens::Vector3 : public yarp::os::idl::WirePortable {
public:
  // Fields
  double c1;
  double c2;
  double c3;

  // Default constructor
  Vector3() : c1(0), c2(0), c3(0) {
  }

  // Constructor with field values
  Vector3(const double c1,const double c2,const double c3) : c1(c1), c2(c2), c3(c3) {
  }

  // Copy constructor
  Vector3(const Vector3& __alt) : WirePortable(__alt)  {
    this->c1 = __alt.c1;
    this->c2 = __alt.c2;
    this->c3 = __alt.c3;
  }

  // Assignment operator
  const Vector3& operator = (const Vector3& __alt) {
    this->c1 = __alt.c1;
    this->c2 = __alt.c2;
    this->c3 = __alt.c3;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);

private:
  bool write_c1(yarp::os::idl::WireWriter& writer);
  bool nested_write_c1(yarp::os::idl::WireWriter& writer);
  bool write_c2(yarp::os::idl::WireWriter& writer);
  bool nested_write_c2(yarp::os::idl::WireWriter& writer);
  bool write_c3(yarp::os::idl::WireWriter& writer);
  bool nested_write_c3(yarp::os::idl::WireWriter& writer);
  bool read_c1(yarp::os::idl::WireReader& reader);
  bool nested_read_c1(yarp::os::idl::WireReader& reader);
  bool read_c2(yarp::os::idl::WireReader& reader);
  bool nested_read_c2(yarp::os::idl::WireReader& reader);
  bool read_c3(yarp::os::idl::WireReader& reader);
  bool nested_read_c3(yarp::os::idl::WireReader& reader);

public:

  yarp::os::ConstString toString();

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<xsens::Vector3 > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new Vector3;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(Vector3& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(Vector3& obj, bool dirty = true) {
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

    Vector3& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_c1(const double c1) {
      will_set_c1();
      obj->c1 = c1;
      mark_dirty_c1();
      communicate();
      did_set_c1();
    }
    void set_c2(const double c2) {
      will_set_c2();
      obj->c2 = c2;
      mark_dirty_c2();
      communicate();
      did_set_c2();
    }
    void set_c3(const double c3) {
      will_set_c3();
      obj->c3 = c3;
      mark_dirty_c3();
      communicate();
      did_set_c3();
    }
    double get_c1() {
      return obj->c1;
    }
    double get_c2() {
      return obj->c2;
    }
    double get_c3() {
      return obj->c3;
    }
    virtual bool will_set_c1() { return true; }
    virtual bool will_set_c2() { return true; }
    virtual bool will_set_c3() { return true; }
    virtual bool did_set_c1() { return true; }
    virtual bool did_set_c2() { return true; }
    virtual bool did_set_c3() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:

    Vector3 *obj;

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
    void mark_dirty_c1() {
      if (is_dirty_c1) return;
      dirty_count++;
      is_dirty_c1 = true;
      mark_dirty();
    }
    void mark_dirty_c2() {
      if (is_dirty_c2) return;
      dirty_count++;
      is_dirty_c2 = true;
      mark_dirty();
    }
    void mark_dirty_c3() {
      if (is_dirty_c3) return;
      dirty_count++;
      is_dirty_c3 = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_c1 = flag;
      is_dirty_c2 = flag;
      is_dirty_c3 = flag;
      dirty_count = flag ? 3 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_c1;
    bool is_dirty_c2;
    bool is_dirty_c3;
  };
};

#endif
