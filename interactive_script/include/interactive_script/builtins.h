#ifndef BUILTINS_H
#define BUILTINS_H

#include "interactive_script/marker_interface.h"
#include "interactive_script/gesture_interface.h"
#include "interactive_script/quad_interface.h"
#include "interactive_script/rosslt_interface.h"
#include "interactive_script/rosslt_demo_interface.h"
#include "interactive_script/tf_interface.h"
#include "luainterpreter.h"
#include "luaparser.h"
#include "interactive_script/async.h"
#include <ui_interactive_script_plugin.h>
#include <QWidget>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

using Gui = Ui::InteractiveScriptWidget;

using TokenMessage = vector<LuaToken>;
Q_DECLARE_METATYPE(TokenMessage)

using SourceChangeMessage = shared_ptr<lua::rt::SourceChange>;
Q_DECLARE_METATYPE(SourceChangeMessage)
Q_DECLARE_METATYPE(QTextCharFormat)

/* The syntax highlighting goes completely haywire in the context of QObjects
 * and VisualizationInterpreter does not include the plugin header for the same
 * reason. To connect the two the signal object is used
*/
struct SignalObject : QObject {
    Q_OBJECT

signals:
    void changeEditorText(QString);
    void applySourceChanges(SourceChangeMessage, QTextCharFormat);
    void clearTerminal();
    void appendTerminal(QString);
    void highlightTokens(TokenMessage);
    void removeFormatting();
    void setBlockValue(QString, QString, QString);
};

struct Interpreter {
    enum class ExecResult {NOERROR, ERR_PARSE, ERR_RUNTIME};

    LuaParser& parser;
    std::shared_ptr<lua::rt::Environment> env = std::make_shared<lua::rt::Environment>(nullptr);

    Interpreter(LuaParser& parser) : parser(parser) {
        env->populate_stdlib();
    }

    ~Interpreter() {
        env->clear();
    }

    std::pair<ExecResult, lua::rt::eval_result_t> dostring(const std::string& program, PerformanceStatistics& ps);
};

struct VisualizationInterpreter {
    VisualizationInterpreter(const std::shared_ptr<rclcpp::Node>& node) : marker(node), rosslt_marker(node), tf(node) {}

    MarkerInterface marker;
    RossltInterface rosslt_marker;
    TfInterface tf;
    SignalObject signal;
    std::atomic<bool> is_running {false};
    LuaParser parser;
    PerformanceStatistics ps;

    void run_script(std::string& script);
    void populate_visualization_env(lua::rt::Environment& env, LuaParser& parser);
};

struct LiveScriptInterpreter {
    LiveScriptInterpreter(const std::shared_ptr<rclcpp::Node>& node) : quad(node), rosslt_demo(node), tf(node), gesture(node) {}

    QuadcopterInterface quad;
    RossltDemoInterface rosslt_demo;
    TfInterface tf;
    GestureInterface gesture;
    Async async;
    SignalObject signal;
    LuaParser parser;

    void run_script(const std::string& script);
    void populate_live_env(lua::rt::Environment& env, const Async::cancel_t& cancelled);
};

#endif // BUILTINS_H
