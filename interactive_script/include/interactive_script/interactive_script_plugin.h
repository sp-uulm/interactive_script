/*
  Copyright 2016 Lucas Walter
*/
#ifndef RQT_GRAPH_EDITOR_H
#define RQT_GRAPH_EDITOR_H

#include "luaparser.h"
#include "luainterpreter.h"
#include <rqt_gui_cpp/plugin.h>
#include <ros/console.h>
#include <ui_interactive_script_plugin.h>
#include <QWidget>
#include <interactive_script/configdialog.h>
#include <memory>
#include <interactive_script/builtins.h>
#include <interactive_script/blocklybridge.h>
#include <interactive_script/load_save_util.h>

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

    void execute_vis();
    void removeFormatting();

    // Comment in to signal that the plugin has a way to configure it
    bool hasConfiguration() const;
    void triggerConfiguration();
    Ui::InteractiveScriptWidget ui_;
private:

    InteractiveScriptSettings settings;

    QWidget* widget_ = nullptr;

    VisualizationInterpreter vis;
    LiveScriptInterpreter live;

    bool eval_paused = false;
    bool is_shutdown = false;
public slots:
    void onChangeEditorText(QString);
    void onApplySourceChanges(SourceChangeMessage msg, QTextCharFormat fmt);
    void onHighlightTokens(TokenMessage, QTextCharFormat);
    void onRemoveFormatting();
    void onClearTerminal();
    void onAppendTerminal(QString);
    void onTextChanged();
    void onRunScriptClicked();
    void updateMarkerInterface() {vis.marker.update();}
    void updateTf();
    void on_save();
    void on_load();
};

}  // namespace rqt_graph_editor
#endif  // RQT_GRAPH_EDITOR_H
