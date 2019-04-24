#ifndef BUILTINS_H
#define BUILTINS_H

#include "interactive_script/marker_interface.h"
#include "luainterpreter.h"
#include "luaparser.h"
#include <ui_interactive_script_plugin.h>
#include <QWidget>

using Gui = Ui::InteractiveScriptWidget;

/* The syntax highlighting goes completely haywire in the context of QObjects
 * and VisualizationInterpreter does not include the plugin header for the same
 * reason. To connect the two the signal object is used
*/
struct SignalObject : QObject {
    Q_OBJECT
signals:
    void changeEditorText(QString);
};

struct VisualizationInterpreter {
    VisualizationInterpreter(Gui *gui) : gui(gui) {}

    Gui *gui;
    MarkerInterface marker;
    LuaChunk parse_result;
    SignalObject signal;

    void run_script();
    void populate_visualization_env(lua::rt::Environment& env, LuaParser& parser);
};

#endif // BUILTINS_H
