#pragma once

#include <string>
#include <QObject>
#include <rviz/display.h>

namespace rviz_plugin_osvr
{

class PlugOSVR : public rviz::Display
{
public:
  PlugOSVR(void); 
  virtual ~PlugOSVR(void);
  virtual void onInitialize();
};

}
