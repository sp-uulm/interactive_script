/*
  Copyright 2016 Lucas Walter
*/

#include "interactive_script/interactive_script_plugin.h"
#include "interactive_script/builtins.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QTimer>

namespace interactive_script_plugin
{

InteractiveScriptGui::InteractiveScriptGui()
  : rqt_gui_cpp::Plugin(), vis(&ui_)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
    setObjectName("InteractiveScript");
}

void InteractiveScriptGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    ui_.terminal->setStyleSheet("background-color: dimgray; color: white");

    connect(ui_.editor, &QPlainTextEdit::textChanged,
            this, &InteractiveScriptGui::onTextChanged);

    connect(&vis.signal, &SignalObject::changeEditorText,
            this, &InteractiveScriptGui::onChangeEditorText);

    setlocale(LC_ALL, "C");
    onTextChanged();

    QTimer* timer = new QTimer(this);
    timer->setInterval(1000/60.0);
    timer->start();
    connect(timer, &QTimer::timeout,
            this,  &InteractiveScriptGui::updateMarkerInterface);
}

void InteractiveScriptGui::shutdownPlugin()
{
  // unregister all publishers here
}

void InteractiveScriptGui::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/,
    qt_gui_cpp::Settings& /*instance_settings*/) const
{
  // instance_settings.setValue(k, v)
}

void InteractiveScriptGui::restoreSettings(const qt_gui_cpp::Settings& /*plugin_settings*/,
    const qt_gui_cpp::Settings& /*instance_settings*/)
{
  // v = instance_settings.value(k)
}

void InteractiveScriptGui::onChangeEditorText(QString s) {
    ui_.editor->setPlainText(s);
}

void InteractiveScriptGui::onTextChanged() {
    vis.run_script();
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace rqt_example_cpp
PLUGINLIB_EXPORT_CLASS(interactive_script_plugin::InteractiveScriptGui, rqt_gui_cpp::Plugin)
