// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <thrift/XsensDriverService.h>
#include <yarp/os/idl/WireTypes.h>

namespace xsens {


class XsensDriverService_calibrate : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class XsensDriverService_calibrateWithType : public yarp::os::Portable {
public:
  std::string calibrationType;
  bool _return;
  void init(const std::string& calibrationType);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class XsensDriverService_startAcquisition : public yarp::os::Portable {
public:
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class XsensDriverService_stopAcquisition : public yarp::os::Portable {
public:
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class XsensDriverService_segments : public yarp::os::Portable {
public:
  std::vector<std::string>  _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class XsensDriverService_bodyDimensions : public yarp::os::Portable {
public:
  std::map<std::string, double>  _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class XsensDriverService_setBodyDimension : public yarp::os::Portable {
public:
  std::string dimensionKey;
  double dimensionValue;
  bool _return;
  void init(const std::string& dimensionKey, const double dimensionValue);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class XsensDriverService_setBodyDimensions : public yarp::os::Portable {
public:
  std::map<std::string, double>  dimensions;
  bool _return;
  void init(const std::map<std::string, double> & dimensions);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool XsensDriverService_calibrate::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("calibrate",1,1)) return false;
  return true;
}

bool XsensDriverService_calibrate::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void XsensDriverService_calibrate::init() {
  _return = false;
}

bool XsensDriverService_calibrateWithType::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("calibrateWithType",1,1)) return false;
  if (!writer.writeString(calibrationType)) return false;
  return true;
}

bool XsensDriverService_calibrateWithType::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void XsensDriverService_calibrateWithType::init(const std::string& calibrationType) {
  _return = false;
  this->calibrationType = calibrationType;
}

bool XsensDriverService_startAcquisition::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("startAcquisition",1,1)) return false;
  return true;
}

bool XsensDriverService_startAcquisition::read(yarp::os::ConnectionReader& connection) {
  YARP_UNUSED(connection);
  return true;
}

void XsensDriverService_startAcquisition::init() {
}

bool XsensDriverService_stopAcquisition::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("stopAcquisition",1,1)) return false;
  return true;
}

bool XsensDriverService_stopAcquisition::read(yarp::os::ConnectionReader& connection) {
  YARP_UNUSED(connection);
  return true;
}

void XsensDriverService_stopAcquisition::init() {
}

bool XsensDriverService_segments::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("segments",1,1)) return false;
  return true;
}

bool XsensDriverService_segments::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  {
    _return.clear();
    uint32_t _size0;
    yarp::os::idl::WireState _etype3;
    reader.readListBegin(_etype3, _size0);
    _return.resize(_size0);
    uint32_t _i4;
    for (_i4 = 0; _i4 < _size0; ++_i4)
    {
      if (!reader.readString(_return[_i4])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return true;
}

void XsensDriverService_segments::init() {
}

bool XsensDriverService_bodyDimensions::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("bodyDimensions",1,1)) return false;
  return true;
}

bool XsensDriverService_bodyDimensions::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  {
    _return.clear();
    uint32_t _size5;
    yarp::os::idl::WireState _ktype6;
    yarp::os::idl::WireState _vtype7;
    reader.readMapBegin(_ktype6, _vtype7, _size5);
    uint32_t _i9;
    for (_i9 = 0; _i9 < _size5; ++_i9)
    {
            uint32_t _size11;
      yarp::os::idl::WireState _lst10;
      reader.readListBegin(_lst10, _size11);
      std::string _key12;
      if (!reader.readString(_key12)) {
        reader.fail();
        return false;
      }
      double& _val13 = _return[_key12];
      if (!reader.readDouble(_val13)) {
        reader.fail();
        return false;
      }
      reader.readListEnd();
    }
    reader.readMapEnd();
  }
  return true;
}

void XsensDriverService_bodyDimensions::init() {
}

bool XsensDriverService_setBodyDimension::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("setBodyDimension",1,1)) return false;
  if (!writer.writeString(dimensionKey)) return false;
  if (!writer.writeDouble(dimensionValue)) return false;
  return true;
}

bool XsensDriverService_setBodyDimension::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void XsensDriverService_setBodyDimension::init(const std::string& dimensionKey, const double dimensionValue) {
  _return = false;
  this->dimensionKey = dimensionKey;
  this->dimensionValue = dimensionValue;
}

bool XsensDriverService_setBodyDimensions::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setBodyDimensions",1,1)) return false;
  {
    if (!writer.writeMapBegin(BOTTLE_TAG_STRING, BOTTLE_TAG_DOUBLE, static_cast<uint32_t>(dimensions.size()))) return false;
    std::map<std::string, double> ::iterator _iter14;
    for (_iter14 = dimensions.begin(); _iter14 != dimensions.end(); ++_iter14)
    {
      if (!writer.writeListBegin(0,2)) return false;
      if (!writer.writeString(_iter14->first)) return false;
      if (!writer.writeDouble(_iter14->second)) return false;
      if (!writer.writeListEnd()) return false;
    }
    if (!writer.writeMapEnd()) return false;
  }
  return true;
}

bool XsensDriverService_setBodyDimensions::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void XsensDriverService_setBodyDimensions::init(const std::map<std::string, double> & dimensions) {
  _return = false;
  this->dimensions = dimensions;
}

XsensDriverService::XsensDriverService() {
  yarp().setOwner(*this);
}
bool XsensDriverService::calibrate() {
  bool _return = false;
  XsensDriverService_calibrate helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool XsensDriverService::calibrate()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool XsensDriverService::calibrateWithType(const std::string& calibrationType) {
  bool _return = false;
  XsensDriverService_calibrateWithType helper;
  helper.init(calibrationType);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool XsensDriverService::calibrateWithType(const std::string& calibrationType)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
void XsensDriverService::startAcquisition() {
  XsensDriverService_startAcquisition helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","void XsensDriverService::startAcquisition()");
  }
  yarp().write(helper);
}
void XsensDriverService::stopAcquisition() {
  XsensDriverService_stopAcquisition helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","void XsensDriverService::stopAcquisition()");
  }
  yarp().write(helper);
}
std::vector<std::string>  XsensDriverService::segments() {
  std::vector<std::string>  _return;
  XsensDriverService_segments helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::vector<std::string>  XsensDriverService::segments()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::map<std::string, double>  XsensDriverService::bodyDimensions() {
  std::map<std::string, double>  _return;
  XsensDriverService_bodyDimensions helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::map<std::string, double>  XsensDriverService::bodyDimensions()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool XsensDriverService::setBodyDimension(const std::string& dimensionKey, const double dimensionValue) {
  bool _return = false;
  XsensDriverService_setBodyDimension helper;
  helper.init(dimensionKey,dimensionValue);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool XsensDriverService::setBodyDimension(const std::string& dimensionKey, const double dimensionValue)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool XsensDriverService::setBodyDimensions(const std::map<std::string, double> & dimensions) {
  bool _return = false;
  XsensDriverService_setBodyDimensions helper;
  helper.init(dimensions);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool XsensDriverService::setBodyDimensions(const std::map<std::string, double> & dimensions)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool XsensDriverService::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "calibrate") {
      bool _return;
      _return = calibrate();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "calibrateWithType") {
      std::string calibrationType;
      if (!reader.readString(calibrationType)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = calibrateWithType(calibrationType);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "startAcquisition") {
      if (!direct) {
        XsensDriverService_startAcquisition helper;
        helper.init();
        yarp().callback(helper,*this,"__direct__");
      } else {
        startAcquisition();
      }
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeOnewayResponse()) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stopAcquisition") {
      if (!direct) {
        XsensDriverService_stopAcquisition helper;
        helper.init();
        yarp().callback(helper,*this,"__direct__");
      } else {
        stopAcquisition();
      }
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeOnewayResponse()) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "segments") {
      std::vector<std::string>  _return;
      _return = segments();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        {
          if (!writer.writeListBegin(BOTTLE_TAG_STRING, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iter15;
          for (_iter15 = _return.begin(); _iter15 != _return.end(); ++_iter15)
          {
            if (!writer.writeString((*_iter15))) return false;
          }
          if (!writer.writeListEnd()) return false;
        }
      }
      reader.accept();
      return true;
    }
    if (tag == "bodyDimensions") {
      std::map<std::string, double>  _return;
      _return = bodyDimensions();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        {
          if (!writer.writeMapBegin(BOTTLE_TAG_STRING, BOTTLE_TAG_DOUBLE, static_cast<uint32_t>(_return.size()))) return false;
          std::map<std::string, double> ::iterator _iter16;
          for (_iter16 = _return.begin(); _iter16 != _return.end(); ++_iter16)
          {
            if (!writer.writeListBegin(0,2)) return false;
            if (!writer.writeString(_iter16->first)) return false;
            if (!writer.writeDouble(_iter16->second)) return false;
            if (!writer.writeListEnd()) return false;
          }
          if (!writer.writeMapEnd()) return false;
        }
      }
      reader.accept();
      return true;
    }
    if (tag == "setBodyDimension") {
      std::string dimensionKey;
      double dimensionValue;
      if (!reader.readString(dimensionKey)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(dimensionValue)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setBodyDimension(dimensionKey,dimensionValue);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setBodyDimensions") {
      std::map<std::string, double>  dimensions;
      {
        dimensions.clear();
        uint32_t _size17;
        yarp::os::idl::WireState _ktype18;
        yarp::os::idl::WireState _vtype19;
        reader.readMapBegin(_ktype18, _vtype19, _size17);
        uint32_t _i21;
        for (_i21 = 0; _i21 < _size17; ++_i21)
        {
                    uint32_t _size23;
          yarp::os::idl::WireState _lst22;
          reader.readListBegin(_lst22, _size23);
          std::string _key24;
          if (!reader.readString(_key24)) {
            reader.fail();
            return false;
          }
          double& _val25 = dimensions[_key24];
          if (!reader.readDouble(_val25)) {
            reader.fail();
            return false;
          }
          reader.readListEnd();
        }
        reader.readMapEnd();
      }
      bool _return;
      _return = setBodyDimensions(dimensions);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> XsensDriverService::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("calibrate");
    helpString.push_back("calibrateWithType");
    helpString.push_back("startAcquisition");
    helpString.push_back("stopAcquisition");
    helpString.push_back("segments");
    helpString.push_back("bodyDimensions");
    helpString.push_back("setBodyDimension");
    helpString.push_back("setBodyDimensions");
    helpString.push_back("help");
  }
  else {
    if (functionName=="calibrate") {
      helpString.push_back("bool calibrate() ");
      helpString.push_back("Calibrate the Xsens device with the default calibration procedure ");
      helpString.push_back("@return true if the calibration is successful, false otherwise ");
    }
    if (functionName=="calibrateWithType") {
      helpString.push_back("bool calibrateWithType(const std::string& calibrationType) ");
      helpString.push_back("Calibrate the Xsens device with the calibration procedure identified ");
      helpString.push_back("by the specified parameter ");
      helpString.push_back("@param calibrationType name of the calibration to execute ");
      helpString.push_back("@return true if the calibration is successful, false otherwise ");
    }
    if (functionName=="startAcquisition") {
      helpString.push_back("void startAcquisition() ");
      helpString.push_back("Start acquiring data from the Xsens suit ");
    }
    if (functionName=="stopAcquisition") {
      helpString.push_back("void stopAcquisition() ");
      helpString.push_back("Stop acquiring data from the suit ");
    }
    if (functionName=="segments") {
      helpString.push_back("std::vector<std::string>  segments() ");
      helpString.push_back("return the segments defined in the Xsens model ");
      helpString.push_back("\note the order of the segments is the same of the ");
      helpString.push_back("one used to output the data ");
      helpString.push_back("@return the list of segment names ");
    }
    if (functionName=="bodyDimensions") {
      helpString.push_back("std::map<std::string, double>  bodyDimensions() ");
      helpString.push_back("returns the body dimensions currently used in the Xsens suit ");
      helpString.push_back("\note these are the dimensions currently used by the Xsens, ");
      helpString.push_back("comprising the one estimated by the device ");
      helpString.push_back("@return all the body dimensions ");
    }
    if (functionName=="setBodyDimension") {
      helpString.push_back("bool setBodyDimension(const std::string& dimensionKey, const double dimensionValue) ");
      helpString.push_back("set the body dimension specified by the key-value pair ");
      helpString.push_back("@param dimensionKey key specifying the dimension to set ");
      helpString.push_back("@param dimensionValue the corresponding dimension value ");
      helpString.push_back("@return true if the set is successful. False otherwise ");
    }
    if (functionName=="setBodyDimensions") {
      helpString.push_back("bool setBodyDimensions(const std::map<std::string, double> & dimensions) ");
      helpString.push_back("set the body dimension specified by the key-value pairs ");
      helpString.push_back("@param dimensions key-value pairs specifying the new dimensions to set ");
      helpString.push_back("@return true if the set is successful. False otherwise ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}
} // namespace


