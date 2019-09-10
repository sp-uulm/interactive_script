#ifndef BUILTINS_H
#define BUILTINS_H

#include "interactive_script/marker_interface.h"
#include "interactive_script/quad_interface.h"
#include "interactive_script/tf_interface.h"
#include "luainterpreter.h"
#include "luaparser.h"
#include <quad_common_utils/async.h>
#include <ui_interactive_script_plugin.h>
#include <QWidget>

using Gui = Ui::InteractiveScriptWidget;

using TokenMessage = vector<LuaToken>;
Q_DECLARE_METATYPE(TokenMessage);

/* The syntax highlighting goes completely haywire in the context of QObjects
 * and VisualizationInterpreter does not include the plugin header for the same
 * reason. To connect the two the signal object is used
*/
struct SignalObject : QObject {
    Q_OBJECT

signals:
    void changeEditorText(QString);
    void clearTerminal();
    void appendTerminal(QString);
    void highlightTokens(TokenMessage);
    void pauseEval(bool);
};

struct Interpreter {
    enum class ExecResult {NOERROR, ERR_PARSE, ERR_RUNTIME};

    LuaParser parser;
    std::shared_ptr<lua::rt::Environment> env = std::make_shared<lua::rt::Environment>(nullptr);

    Interpreter() {
        env->populate_stdlib();
    }

    ~Interpreter() {
        env->clear();
    }

    std::pair<ExecResult, std::string> dostring(const std::string& program);
};

struct VisualizationInterpreter {
    MarkerInterface marker;
    TfInterface tf;
    SignalObject signal;
    std::atomic<bool> is_running;

    void run_script(const std::string& script);
    void populate_visualization_env(lua::rt::Environment& env, LuaParser& parser);
};

struct LiveScriptInterpreter {
    LiveScriptInterpreter() {}

    QuadcopterInterface quad;
    TfInterface tf;
    Async async;
    SignalObject signal;

    void run_script(const std::string& script);
    void populate_live_env(lua::rt::Environment& env, const Async::cancel_t& cancelled);
};

#endif // BUILTINS_H
