/*
  Copyright 2016 Lucas Walter
*/
#ifndef RQT_GRAPH_EDITOR_H
#define RQT_GRAPH_EDITOR_H

#include "luaparser.h"
#include "luainterpreter.h"
#include <rqt_gui_cpp/plugin.h>
#include <ui_interactive_script_plugin.h>
#include <QWidget>
#include <memory>
#include <interactive_script/builtins.h>

namespace interactive_script_plugin
{

class InteractiveScriptGui
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  InteractiveScriptGui();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
  Ui::InteractiveScriptWidget ui_;
private:
  QWidget* widget_ = nullptr;

  VisualizationInterpreter vis;

public slots:
    void onChangeEditorText(QString);
    void onTextChanged();
    void updateMarkerInterface() {vis.marker.update();}
};

}  // namespace rqt_graph_editor
#endif  // RQT_GRAPH_EDITOR_H
