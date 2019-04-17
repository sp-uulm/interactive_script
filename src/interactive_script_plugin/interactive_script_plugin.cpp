/*
  Copyright 2016 Lucas Walter
*/

#include "interactive_script/interactive_script_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QTimer>
#include <sstream>

namespace interactive_script_plugin
{

InteractiveScriptGui::InteractiveScriptGui()
  : rqt_gui_cpp::Plugin()
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

    connect(this, &InteractiveScriptGui::changeEditorText,
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
    LuaParser parser;
    const auto result = parser.parse(ui_.editor->toPlainText().toStdString());

    if (holds_alternative<string>(result)) {
        ui_.terminal->setPlainText(QString::fromStdString("Error: " + get<string>(result)));
        parse_result.reset();
    } else {
        parse_result = get<LuaChunk>(result);

        auto env = make_shared<lua::rt::Environment>(nullptr);

        env->populate_stdlib();

        env->assign(string {"print"}, make_shared<lua::rt::cfunction>([this](const lua::rt::vallist& args) -> lua::rt::vallist {
            stringstream ss;
            for (int i = 0; i < static_cast<int>(args.size()) - 1; ++i) {
                ss << args[i].to_string() << "\t";
            }
            if (args.size() > 0) {
                ss << args.back().to_string();
            }
            ui_.terminal->appendPlainText(QString::fromStdString(ss.str()));
            return {};
        }), false);

        env->assign(string {"point"}, make_shared<lua::rt::cfunction>([this, &parser](const lua::rt::vallist& args) -> lua::rt::vallist {
            if (args.size() != 3 || args[0].type() != "number" || args[1].type() != "number" || args[2].type() != "number") {
                ui_.terminal->appendPlainText("point requires 3 number arguments");
                return {lua::rt::nil()};
            }
            marker.addPoint(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]),
                                        args[0].source.get(), args[1].source.get(), args[2].source.get(),
                [x_source = args[0].source, y_source = args[1].source, z_source = args[2].source, this, tokens = parser.tokens](const auto& feedback) mutable {
                    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
                        lua::rt::SourceChangeAnd changes;
                        if (x_source && feedback->control_name == "move_x") {
                            if (const auto& change = x_source->forceValue(feedback->pose.position.x))
                                changes.changes.push_back(*change);
                        }
                        if (y_source && feedback->control_name == "move_y") {
                            if (const auto& change = y_source->forceValue(feedback->pose.position.y))
                                changes.changes.push_back(*change);
                        }
                        if (z_source && feedback->control_name == "move_z") {
                            if (const auto& change = z_source->forceValue(feedback->pose.position.z))
                                changes.changes.push_back(*change);
                        }
                        changes.apply(tokens);
                        emit changeEditorText(QString::fromStdString(get_string(tokens)));
                    }
                });
            return {};
        }), false);

        env->assign(string {"line"}, make_shared<lua::rt::cfunction>([this](const lua::rt::vallist& args) -> lua::rt::vallist {
            if (args.size() != 6
                    || args[0].type() != "number" || args[1].type() != "number" || args[2].type() != "number"
                    || args[3].type() != "number" || args[4].type() != "number" || args[5].type() != "number") {
                ui_.terminal->appendPlainText("line requires 6 number arguments");
                return {lua::rt::nil()};
            }
            marker.addLine(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]),
                           get<double>(args[3]), get<double>(args[4]), get<double>(args[5]));
            return {};
        }), false);

        ui_.terminal->setPlainText("");
        lua::rt::ASTEvaluator eval;
        if (auto eval_result = parse_result->accept(eval, env); holds_alternative<string>(eval_result)) {
            ui_.terminal->setPlainText(QString::fromStdString("Error: " + get<string>(eval_result)));
        } else {
            marker.commit();
        }

        env->clear();
    }
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
