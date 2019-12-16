/*
  Copyright 2016 Lucas Walter
*/
#ifndef RQT_GRAPH_EDITOR_H
#define RQT_GRAPH_EDITOR_H

#include "luaparser.h"
#include "luainterpreter.h"
#include <rqt_gui_cpp/plugin.h>
#include <rclcpp/rclcpp.hpp>
#include <ui_interactive_script_plugin.h>
#include <QWidget>
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
    virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
    virtual void shutdownPlugin() override;
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const override;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings) override;

    void execute_vis();
    void removeFormatting();

    // Comment in to signal that the plugin has a way to configure it
    // bool hasConfiguration() const;
    // void triggerConfiguration();
    Ui::InteractiveScriptWidget ui_;
private:
    bool setting_print_performance_statistics = true;

    QWidget* widget_ = nullptr;

    std::unique_ptr<VisualizationInterpreter> vis;
    std::unique_ptr<LiveScriptInterpreter> live;

    bool eval_paused = false;
    bool is_shutdown = false;
public slots:
    void onChangeEditorText(QString);
    void onApplySourceChanges(SourceChangeMessage msg, QTextCharFormat fmt);
    void onHighlightTokens(TokenMessage);
    void onRemoveFormatting();
    void onClearTerminal();
    void onAppendTerminal(QString);
    void onTextChanged();
    void onRunScriptClicked();
    void updateMarkerInterface() {vis->marker.update();}
    void updateTf();
    void on_save();
    void on_load();
};

}  // namespace rqt_graph_editor
#endif  // RQT_GRAPH_EDITOR_H
