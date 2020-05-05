#include "interactive_script/builtins.h"
#include "interactive_script/interactive_script_plugin.h"

#include <sstream>
#include <string>
#include <variant>
#include <chrono>
#include <QString>

using namespace lua::rt;
using namespace std;

std::pair<Interpreter::ExecResult, eval_result_t> Interpreter::dostring(const string& program, PerformanceStatistics& ps) {
    const auto result = parser.parse(program, ps);
    auto parse_end = chrono::steady_clock::now();

    if (holds_alternative<string>(result)) {
        return {ExecResult::ERR_PARSE, get<string>(result)};
    } else {
        auto parse_result = get<LuaChunk>(result);

        lua::rt::ASTEvaluator eval;
        if (auto eval_result = parse_result->accept(eval, env); holds_alternative<string>(eval_result)) {
            return {ExecResult::ERR_RUNTIME, eval_result};
        } else {
            auto eval_end = chrono::steady_clock::now();
            ps.execute = chrono::duration_cast<chrono::microseconds>(eval_end - parse_end);
            return {ExecResult::NOERROR, eval_result};
        }
    }
}

QTextCharFormat format(Qt::GlobalColor bg_color, Qt::GlobalColor fg_color = Qt::white) {
    QTextCharFormat fmt;
    fmt.setBackground(bg_color);
    fmt.setForeground(fg_color);
    return fmt;
}

void VisualizationInterpreter::run_script(std::string& script, const InteractiveScriptSettings& settings) {
    if (is_running.exchange(true)) { // avoid running scripts in parallel
        return;
    }

    signal.clearTerminal();
    marker.runtime = chrono::duration<double>();

    auto exec_start = chrono::steady_clock::now();
    Interpreter interpreter {parser};
    populate_visualization_env(*interpreter.env, interpreter.parser, settings);
    auto exec_ready = chrono::steady_clock::now();

    switch (auto [c, s] = interpreter.dostring(script, ps); c) {
    case Interpreter::ExecResult::ERR_PARSE:
        signal.appendTerminal(QString::fromStdString(get<string>(s)));
        break;
    case Interpreter::ExecResult::ERR_RUNTIME:
        signal.appendTerminal(QString::fromStdString(get<string>(s)));
        break;
    case Interpreter::ExecResult::NOERROR:
        {
            // apply the source changes
            auto exec_end = chrono::steady_clock::now();
            auto sc = get_sc(s);
            if (sc) {
                QTextCharFormat fmt;
                fmt.setBackground(Qt::red);
                fmt.setForeground(Qt::white);
                signal.applySourceChanges(*sc, fmt);
            }

            marker.commit();

            auto sc_end = chrono::steady_clock::now();

            ps.total = chrono::duration_cast<chrono::microseconds>(sc_end - exec_start);
            ps.create_env = chrono::duration_cast<chrono::microseconds>(exec_ready - exec_start);
            ps.marker_interface = chrono::duration_cast<chrono::microseconds>(marker.runtime);
            ps.source_changes = chrono::duration_cast<chrono::microseconds>(sc_end - exec_end);
            break;
        }
    }

    is_running.store(false);
}

void VisualizationInterpreter::populate_visualization_env(Environment& env, LuaParser& parser, const InteractiveScriptSettings& settings) {
    env.assign("__quad_pose", make_shared<table>(
        vector<pair<val, val>> {{"x", 0},{"y", 0},{"z", 0},{"psi", 0}}), false);

    env.assign("__quad_target", make_shared<table>(
        vector<pair<val, val>> {{"x", 0},{"y", 0},{"z", 0},{"psi", 0}}), false);

    env.assign("print", make_shared<lua::rt::cfunction>([this](const lua::rt::vallist& args) -> cfunction::result {
        stringstream ss;
        for (int i = 0; i < static_cast<int>(args.size()) - 1; ++i) {
            ss << args[static_cast<unsigned>(i)].to_string() << "\t";
        }
        if (!args.empty()) {
            ss << args.back().to_string();
        }
        signal.appendTerminal(QString::fromStdString(ss.str()));
        return {};
    }), false);

    env.assign("moveTo", make_shared<cfunction>([this, &env, &parser, settings](const vallist& args) mutable -> cfunction::result {
        if (args.size() == 4 && args[0].isnumber() && args[1].isnumber() && args[2].isnumber() && args[3].isnumber()) {
            val target = env.getvar("__quad_target");
            table& tab_target = *get<table_p>(target);

            val pose = env.getvar("__quad_pose");
            table& tab_pose = *get<table_p>(pose);


            marker.addPose(get<double>(args[0]),
                           get<double>(args[1]),
                           get<double>(args[2]),
                           get<double>(args[3]),
                           args[0].source.get() && settings.move_markers,
                           args[1].source.get() && settings.move_markers,
                           args[2].source.get() && settings.move_markers,
                           args[3].source.get() && settings.move_markers,
                    [x_source = args[0].source, y_source = args[1].source, z_source = args[2].source, psi_source = args[3].source,
                     this, settings, tokens = parser.tokens, original_psi = get<double>(args[3])](const auto& feedback) mutable {
                        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK && settings.click_for_dependency_trace) {
                            vector<LuaToken> tokens;
                            signal.removeFormatting();

                            if(x_source) {
                                signal.highlightTokens(x_source->get_all_tokens(), format(Qt::red));
                            }

                            if(y_source) {
                                signal.highlightTokens(y_source->get_all_tokens(), format(Qt::darkGreen));
                            }

                            if(z_source) {
                                signal.highlightTokens(z_source->get_all_tokens(), format(Qt::blue));
                            }

                            if(psi_source) {
                                signal.highlightTokens(psi_source->get_all_tokens(), format(Qt::darkCyan));
                            }

                        } else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
                            if (feedback->control_name != "clicked") {
                                signal.removeFormatting();
                            }

                            auto changes = make_shared<lua::rt::SourceChangeAnd>();
                            if (x_source && feedback->control_name == "move_x") {
                                if (const auto& change = x_source->forceValue(feedback->pose.position.x))
                                    changes->changes.push_back(*change);
                            }
                            if (y_source && feedback->control_name == "move_y") {
                                if (const auto& change = y_source->forceValue(feedback->pose.position.y))
                                    changes->changes.push_back(*change);
                            }
                            if (z_source && feedback->control_name == "move_z") {
                                if (const auto& change = z_source->forceValue(feedback->pose.position.z))
                                    changes->changes.push_back(*change);
                            }
                            if (psi_source && feedback->control_name == "move_psi") {
                                if (const auto& change = psi_source->forceValue(fmod(yaw(feedback->pose.orientation) + original_psi, 2*M_PI)))
                                    changes->changes.push_back(*change);
                            }

                            // apply and highlight changes
                            if (feedback->control_name != "clicked") {
                                QTextCharFormat fmt;
                                fmt.setBackground(Qt::red);
                                fmt.setForeground(Qt::white);
                                signal.applySourceChanges(changes, fmt);
                            }
                        }
                    });

            marker.addLine(tab_pose["x"].def_number(),
                           tab_pose["y"].def_number(),
                           tab_pose["z"].def_number(),
                           get<double>(args[0]),
                           get<double>(args[1]),
                           get<double>(args[2]),
                           MarkerInterface::Color::RED,
                           0.02);

            tab_target["x"] = args[0];
            tab_target["y"] = args[1];
            tab_target["z"] = args[2];
            tab_target["psi"] = args[3];

            return {};
        }

        signal.appendTerminal("invalid args to moveTo(x, y, z, psi)");
        return {nil()};
    }), false);

    env.assign("sleep", make_shared<lua::rt::cfunction>([this, &env](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 1 || !args[0].isnumber()) {
            signal.appendTerminal("sleep requires a number argument");
            return {nil()};
        }

        val pose = env.getvar("__quad_pose");
        table& tab_pose = *get<table_p>(pose);

        val target = env.getvar("__quad_target");
        table& tab_target = *get<table_p>(target);

        val px = tab_pose["x"].def_number();
        val py = tab_pose["y"].def_number();
        val pz = tab_pose["z"].def_number();
        val tx = tab_target["x"];
        val ty = tab_target["y"];
        val tz = tab_target["z"];

        try {
            val length = unwrap(op_sqrt(
                      ((tx - px)^2)
                    + ((ty - py)^2)
                    + ((tz - pz)^2)));

            val duration = length / 1.0; // 1 m/s

            tab_pose["x"] = (tx - px) * clamp(args[0]/duration, val(0.0), val(1.0)) + px;
            tab_pose["y"] = (ty - py) * clamp(args[0]/duration, val(0.0), val(1.0)) + py;
            tab_pose["z"] = (tz - pz) * clamp(args[0]/duration, val(0.0), val(1.0)) + pz;
            tab_pose["psi"] = tab_target["psi"];
        } catch (const runtime_error& err) {
            return string{err.what()};
        }

        marker.addLine(get<double>(px),
                       get<double>(py),
                       get<double>(pz),
                       tab_pose["x"].def_number(),
                       tab_pose["y"].def_number(),
                       tab_pose["z"].def_number());

        return {};
    }), false);

    env.assign("wait", make_shared<lua::rt::cfunction>([this, &env](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 0) {
            signal.appendTerminal("wait requires no arguments");
            return {nil()};
        }

        val pose = env.getvar("__quad_pose");
        table& tab_pose = *get<table_p>(pose);

        val target = env.getvar("__quad_target");
        table& tab_target = *get<table_p>(target);

        marker.addLine(tab_pose["x"].def_number(),
                       tab_pose["y"].def_number(),
                       tab_pose["z"].def_number(),
                       tab_target["x"].def_number(),
                       tab_target["y"].def_number(),
                       tab_target["z"].def_number());

        tab_pose["x"] = tab_target["x"];
        tab_pose["y"] = tab_target["y"];
        tab_pose["z"] = tab_target["z"];
        tab_pose["psi"] = tab_target["psi"];

        return {};
    }), false);

    env.assign("pose", make_shared<cfunction>([this, &env](const vallist& args) mutable -> cfunction::result {
        if (args.size() == 0) {
            return {env.getvar("__quad_pose")};
        }

        if (args.size() == 1 && args[0].isstring()) {
            auto p = tf.get_pose(get<string>(args[0]));

            val result = make_shared<table>();
            table& t = *get<table_p>(result);
            t["x"] = p ? p->position.x : 0;
            t["y"] = p ? p->position.y : 0;
            t["z"] = p ? p->position.z : 0;
            t["psi"] = p ? yaw(p->orientation) : 0;

            return {result};
        }

        signal.appendTerminal("invalid args to pose(object?)");
        return {nil()};
    }), false);

    env.assign("blockValue", make_shared<cfunction>([this, &env](const vallist& args) mutable -> cfunction::result {
        if (args.size() != 3 || !args[0].isstring() || !args[1].isstring()) {
            signal.appendTerminal("invalid args to blockValue(id, field, val)");
            return {nil()};
        }

        val result = args[2];

        // set source
        struct block_val : sourceexp {
            block_val(SignalObject& signal, const string& id, const string& field) : signal(signal), id(id), field(field) {}

            optional<shared_ptr<SourceChange>> forceValue(const val& newval) const override{
                // TODO: unsauber, SourceChangeBlock als return!
                signal.setBlockValue(QString::fromStdString(id),
                                     QString::fromStdString(field),
                                     QString::fromStdString(newval.to_string()));

                signal.highlightField(QString::fromStdString(id), QString::fromStdString(field));

                return nullopt;
            }

            eval_result_t reevaluate() override {
                return string{"not implemented"};
            }

            bool isDirty() const override {
                return false;
            }

            vector<LuaToken> get_all_tokens() const override {
                // TODO: remove this hack

                signal.highlightField(QString::fromStdString(id), QString::fromStdString(field));

                return {};
            }

            SignalObject& signal;
            string id;
            string field;
        };

        result.source = std::make_shared<block_val>(signal,
                                                    get<string>(args[0]),
                                                    get<string>(args[1]));

        return {result};
    }), false);

    auto rviz = make_shared<table>();
    env.assign("rviz", rviz, false);

    (*rviz)["point"] = make_shared<lua::rt::cfunction>([this, settings, &parser](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 3 || !args[0].isnumber() || !args[1].isnumber() || !args[2].isnumber()) {
            signal.appendTerminal("point requires 3 number arguments");
            return {nil()};
        }
        marker.addPoint(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]),
                        args[0].source.get() && settings.move_markers,
                        args[1].source.get() && settings.move_markers,
                        args[2].source.get() && settings.move_markers,
            [x_source = args[0].source, y_source = args[1].source, z_source = args[2].source, this, settings, tokens = parser.tokens](const auto& feedback) mutable {
                if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK && settings.click_for_dependency_trace) {
                    vector<LuaToken> tokens;
                    signal.removeFormatting();

                    if(x_source) {
                        signal.highlightTokens(x_source->get_all_tokens(), format(Qt::red));
                    }

                    if(y_source) {
                        signal.highlightTokens(y_source->get_all_tokens(), format(Qt::darkGreen));
                    }

                    if(z_source) {
                        signal.highlightTokens(z_source->get_all_tokens(), format(Qt::blue));
                    }

                } else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
                    if (feedback->control_name != "clicked") {
                        signal.removeFormatting();
                    }

                    auto changes = make_shared<lua::rt::SourceChangeAnd>();
                    if (x_source && feedback->control_name == "move_x") {
                        if (const auto& change = x_source->forceValue(feedback->pose.position.x))
                            changes->changes.push_back(*change);
                    }
                    if (y_source && feedback->control_name == "move_y") {
                        if (const auto& change = y_source->forceValue(feedback->pose.position.y))
                            changes->changes.push_back(*change);
                    }
                    if (z_source && feedback->control_name == "move_z") {
                        if (const auto& change = z_source->forceValue(feedback->pose.position.z))
                            changes->changes.push_back(*change);
                    }

                    // apply and highlight changes
                    if (feedback->control_name != "clicked") {
                        QTextCharFormat fmt;
                        fmt.setBackground(Qt::red);
                        fmt.setForeground(Qt::white);
                        signal.applySourceChanges(changes, fmt);
                    }
                }
            });
        return {};
    });

    (*rviz)["line"] = make_shared<lua::rt::cfunction>([this](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 6
                || !args[0].isnumber() || !args[1].isnumber() || !args[2].isnumber()
                || !args[3].isnumber() || !args[4].isnumber() || !args[5].isnumber()) {
            signal.appendTerminal("line requires 6 number arguments");
            return {nil()};
        }
        marker.addLine(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]),
                            get<double>(args[3]), get<double>(args[4]), get<double>(args[5]));
        return {};
    });

    (*rviz)["pose"] = make_shared<lua::rt::cfunction>([this, settings, &parser](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 4 || !args[0].isnumber() || !args[1].isnumber() || !args[2].isnumber() || !args[3].isnumber()) {
            signal.appendTerminal("pose requires 4 number arguments");
            return {nil()};
        }
        marker.addPose(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]), get<double>(args[3]),
                       args[0].source.get() && settings.move_markers,
                       args[1].source.get() && settings.move_markers,
                       args[2].source.get() && settings.move_markers,
                       args[3].source.get() && settings.move_markers,
            [x_source = args[0].source, y_source = args[1].source, z_source = args[2].source, psi_source = args[3].source, this, settings, tokens = parser.tokens, original_psi = get<double>(args[3])](const auto& feedback) mutable {
                if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK && settings.click_for_dependency_trace) {
                    vector<LuaToken> tokens;
                    signal.removeFormatting();

                    if(x_source) {
                        signal.highlightTokens(x_source->get_all_tokens(), format(Qt::red));
                    }

                    if(y_source) {
                        signal.highlightTokens(y_source->get_all_tokens(), format(Qt::darkGreen));
                    }

                    if(z_source) {
                        signal.highlightTokens(z_source->get_all_tokens(), format(Qt::blue));
                    }

                    if(psi_source) {
                        signal.highlightTokens(psi_source->get_all_tokens(), format(Qt::darkCyan));
                    }

                } else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
                    if (feedback->control_name != "clicked") {
                        signal.removeFormatting();
                    }

                    auto changes = make_shared<lua::rt::SourceChangeAnd>();
                    if (x_source && feedback->control_name == "move_x") {
                        if (const auto& change = x_source->forceValue(feedback->pose.position.x))
                            changes->changes.push_back(*change);
                    }
                    if (y_source && feedback->control_name == "move_y") {
                        if (const auto& change = y_source->forceValue(feedback->pose.position.y))
                            changes->changes.push_back(*change);
                    }
                    if (z_source && feedback->control_name == "move_z") {
                        if (const auto& change = z_source->forceValue(feedback->pose.position.z))
                            changes->changes.push_back(*change);
                    }
                    if (psi_source && feedback->control_name == "move_psi") {
                        if (const auto& change = psi_source->forceValue(fmod(yaw(feedback->pose.orientation) + original_psi, 2*M_PI)))
                            changes->changes.push_back(*change);
                    }

                    // apply and highlight changes
                    if (feedback->control_name != "clicked") {
                        QTextCharFormat fmt;
                        fmt.setBackground(Qt::red);
                        fmt.setForeground(Qt::white);
                        signal.applySourceChanges(changes, fmt);
                    }
                }
            });
        return {};
    });
}

void LiveScriptInterpreter::run_script(const std::string& script, const InteractiveScriptSettings&) {
    async = Async([this, script](const Async::cancel_t& cancelled){
        Interpreter interpreter {parser};

        signal.clearTerminal();
        populate_live_env(*interpreter.env, cancelled);
        PerformanceStatistics ps;
        switch (auto [c, s] = interpreter.dostring(script, ps); c) {
        case Interpreter::ExecResult::ERR_PARSE:
            signal.appendTerminal(QString::fromStdString(get<string>(s)));
            break;
        case Interpreter::ExecResult::ERR_RUNTIME:
            signal.appendTerminal(QString::fromStdString(get<string>(s)));
            break;
        case Interpreter::ExecResult::NOERROR:
            signal.appendTerminal("Info: Execution finished");
        }
    });
}

void LiveScriptInterpreter::populate_live_env(lua::rt::Environment &env, const Async::cancel_t& cancelled) {
    env.assign("print", make_shared<lua::rt::cfunction>([this, cancelled](const lua::rt::vallist& args) -> cfunction::result {
        if (*cancelled)
            return string {"Script execution cancelled!"};

        stringstream ss;
        for (int i = 0; i < static_cast<int>(args.size()) - 1; ++i) {
            ss << args[static_cast<unsigned>(i)].to_string() << "\t";
        }
        if (!args.empty()) {
            ss << args.back().to_string();
        }
        cout << ss.str() << endl;
        signal.appendTerminal(QString::fromStdString(ss.str()));
        return {};
    }), false);

    env.assign(string {"sleep"}, make_shared<lua::rt::cfunction>([this, cancelled](const lua::rt::vallist& args, const _LuaFunctioncall stmt) -> cfunction::result {
        if (*cancelled)
            return string {"Script execution cancelled!"};

        if (args.size() != 1 || !args[0].isnumber()) {
            signal.appendTerminal("sleep requires a number argument");
            return {nil()};
        }

        signal.highlightTokens(stmt.tokens, format(Qt::magenta));

        std::this_thread::sleep_for(chrono::milliseconds(static_cast<long>(get<double>(args[0])*1000)));

        return {};
    }), false);

    env.assign(string {"wait"}, make_shared<lua::rt::cfunction>([this, cancelled](const lua::rt::vallist& args, const _LuaFunctioncall stmt) -> cfunction::result {
        if (*cancelled)
            return string {"Script execution cancelled!"};

        if (args.size() != 0) {
            signal.appendTerminal("wait requires no arguments");
            return {nil()};
        }

        signal.highlightTokens(stmt.tokens, format(Qt::magenta));

        while (!quad.is_at_target()) {
            if (*cancelled)
                return string {"Script execution cancelled!"};
            std::this_thread::sleep_for(chrono::milliseconds(20));
        }

        return {};
    }), false);

    env.assign(string {"moveTo"}, make_shared<lua::rt::cfunction>([this, cancelled](const lua::rt::vallist& args) -> cfunction::result {
        if (*cancelled)
            return string {"Script execution cancelled!"};

        if (args.size() == 4 && args[0].isnumber() && args[1].isnumber() && args[2].isnumber() && args[3].isnumber()) {
            quad.send_new_waypoint(geometry_msgs::pose(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]), get<double>(args[3])));

            return {};
        }

        signal.appendTerminal("invalid args to moveTo(x, y, z, psi)");
        return {nil()};
    }), false);

    env.assign(string {"pose"}, make_shared<lua::rt::cfunction>([this, cancelled](const lua::rt::vallist& args) -> cfunction::result {
        if (*cancelled)
            return string {"Script execution cancelled!"};

        if (args.size() == 0) {
            auto p = quad.get_current_pose();
            table_p t = make_shared<table>();
            (*t)["x"] = p.position.x;
            (*t)["y"] = p.position.y;
            (*t)["z"] = p.position.x;
            (*t)["psi"] = yaw(p.orientation);

            return {t};
        }

        if (args.size() == 1 && args[0].isstring()) {
            auto p = geometry_msgs::origin();

            if (auto _p = tf.get_pose(get<string>(args[0]))) {
                p = *_p;
            } else {
                signal.appendTerminal("invalid frame for pose. Is tf connected?");
                return {nil()};
            }

            table_p t = make_shared<table>();
            (*t)["x"] = p.position.x;
            (*t)["y"] = p.position.y;
            (*t)["z"] = p.position.x;
            (*t)["psi"] = yaw(p.orientation);

            return {t};
        }

        signal.appendTerminal("invalid args to pose(object?)");
        return {nil()};
    }), false);

    env.assign("blockValue", make_shared<cfunction>([this, &env](const vallist& args) mutable -> cfunction::result {
        if (args.size() != 3 || !args[0].isstring() || !args[1].isstring()) {
            signal.appendTerminal("invalid args to blockValue(id, field, val)");
            return {nil()};
        }

        return {args[2]};
    }), false);

}
