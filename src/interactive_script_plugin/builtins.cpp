#include "interactive_script/builtins.h"
#include "interactive_script/interactive_script_plugin.h"

#include <sstream>
#include <string>
#include <variant>
#include <chrono>
#include <QString>

using namespace lua::rt;
using namespace std;

std::pair<Interpreter::ExecResult, std::string> Interpreter::dostring(const string& program) {
    const auto result = parser.parse(program);

    if (holds_alternative<string>(result)) {
        return {ExecResult::ERR_PARSE, get<string>(result)};
    } else {
        auto parse_result = get<LuaChunk>(result);

        lua::rt::ASTEvaluator eval;
        if (auto eval_result = parse_result->accept(eval, env); holds_alternative<string>(eval_result)) {
            return {ExecResult::ERR_RUNTIME, get<string>(eval_result)};
        }
        return {ExecResult::NOERROR, ""};
    }
}

void VisualizationInterpreter::run_script(const std::string& script) {
    signal.clearTerminal();

    Interpreter interpreter;
    populate_visualization_env(*interpreter.env, interpreter.parser);

    switch (auto [c, s] = interpreter.dostring(script); c) {
    case Interpreter::ExecResult::ERR_PARSE:
        signal.appendTerminal(QString::fromStdString(s));
        break;
    case Interpreter::ExecResult::ERR_RUNTIME:
        signal.appendTerminal(QString::fromStdString(s));
    case Interpreter::ExecResult::NOERROR:
        marker.commit();
    }
}

void VisualizationInterpreter::populate_visualization_env(Environment& env, LuaParser& parser) {
    env.assign(string {"print"}, make_shared<lua::rt::cfunction>([this](const lua::rt::vallist& args) -> cfunction::result {
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

    env.assign(string {"moveTo"}, make_shared<cfunction>([this, &env, &parser](const vallist& args) mutable -> cfunction::result {
        if (args.size() == 4 && args[0].isnumber() && args[1].isnumber() && args[2].isnumber() && args[3].isnumber()) {
            val target = env.getvar(string{"__quad_target"});
            if (target.type() != "table") {
                target = make_shared<table>();
                env.assign(string{"__quad_target"}, target, false);
            }

            table& tab_target = *get<table_p>(target);

            val pose = env.getvar(string{"__quad_pose"});
            if (pose.type() != "table") {
                pose = make_shared<table>();
                env.assign(string{"__quad_pose"}, pose, false);
            }

            table& tab_pose = *get<table_p>(pose);


            marker.addPose(get<double>(args[0]),
                           get<double>(args[1]),
                           get<double>(args[2]),
                           get<double>(args[3]),
                           args[0].source.get(),
                           args[1].source.get(),
                           args[2].source.get(),
                           args[3].source.get(),
                    [x_source = args[0].source, y_source = args[1].source, z_source = args[2].source, psi_source = args[3].source, this, tokens = parser.tokens, original_psi = get<double>(args[3])](const auto& feedback) mutable {
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
                            if (psi_source && feedback->control_name == "move_psi") {
                                if (const auto& change = psi_source->forceValue(fmod(yaw(feedback->pose.orientation) + original_psi, 2*M_PI)))
                                    changes.changes.push_back(*change);
                            }

                            // make sure the script is only executed once after all highlights are set
                            signal.pauseEval(true);
                            auto modified_tokens = changes.apply(tokens);
                            signal.changeEditorText(QString::fromStdString(get_string(tokens)));
                            signal.highlightTokens(modified_tokens);
                            signal.pauseEval(false);
                        }
                    });

            marker.addLine(tab_pose[string{"x"}].def_number(),
                           tab_pose[string{"y"}].def_number(),
                           tab_pose[string{"z"}].def_number(),
                           get<double>(args[0]),
                           get<double>(args[1]),
                           get<double>(args[2]),
                           MarkerInterface::Color::RED,
                           0.02);

            tab_target[string{"x"}] = args[0];
            tab_target[string{"y"}] = args[1];
            tab_target[string{"z"}] = args[2];
            tab_target[string{"psi"}] = args[3];

            return {};
        }

        signal.appendTerminal("invalid args to moveTo(x, y, z, psi)");
        return {nil()};
    }), false);

    env.assign(string {"sleep"}, make_shared<lua::rt::cfunction>([this, &env](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 1 || !args[0].isnumber()) {
            signal.appendTerminal("sleep requires a number argument");
            return {nil()};
        }

        val pose = env.getvar(string{"__quad_pose"});
        if (pose.type() != "table") {
            pose = make_shared<table>();
            env.assign(string{"__quad_pose"}, pose, false);
        }

        table& tab_pose = *get<table_p>(pose);

        val target = env.getvar(string{"__quad_target"});
        if (target.type() != "table") {
            target = make_shared<table>();
            env.assign(string{"__quad_target"}, target, false);
        }

        table& tab_target = *get<table_p>(target);

        val px = tab_pose[string{"x"}].def_number();
        val py = tab_pose[string{"y"}].def_number();
        val pz = tab_pose[string{"z"}].def_number();
        val tx = tab_target[string{"x"}];
        val ty = tab_target[string{"y"}];
        val tz = tab_target[string{"z"}];

        try {
            val length = unwrap(op_sqrt(
                      ((tx - px)^2.0)
                    + ((ty - py)^2.0)
                    + ((tz - pz)^2.0)));

            val duration = length / 1.0; // 1 m/s

            tab_pose[string{"x"}] = (tx - px) * clamp(args[0]/duration, val(0.0), val(1.0)) + px;
            tab_pose[string{"y"}] = (ty - py) * clamp(args[0]/duration, val(0.0), val(1.0)) + py;
            tab_pose[string{"z"}] = (tz - pz) * clamp(args[0]/duration, val(0.0), val(1.0)) + pz;
            tab_pose[string{"psi"}] = tab_target[string{"psi"}];
        } catch (const runtime_error& err) {
            return string{err.what()};
        }

        marker.addLine(get<double>(px),
                       get<double>(py),
                       get<double>(pz),
                       tab_pose[string{"x"}].def_number(),
                       tab_pose[string{"y"}].def_number(),
                       tab_pose[string{"z"}].def_number());

        return {};
    }), false);

    env.assign(string {"wait"}, make_shared<lua::rt::cfunction>([this, &env](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 0) {
            signal.appendTerminal("wait requires no arguments");
            return {nil()};
        }

        val pose = env.getvar(string{"__quad_pose"});
        if (pose.type() != "table") {
            pose = make_shared<table>();
            env.assign(string{"__quad_pose"}, pose, false);
        }

        table& tab_pose = *get<table_p>(pose);

        val target = env.getvar(string{"__quad_target"});
        if (target.type() != "table") {
            target = make_shared<table>();
            env.assign(string{"__quad_target"}, target, false);
        }

        table& tab_target = *get<table_p>(target);

        marker.addLine(tab_pose[string{"x"}].def_number(),
                       tab_pose[string{"y"}].def_number(),
                       tab_pose[string{"z"}].def_number(),
                       tab_target[string{"x"}].def_number(),
                       tab_target[string{"y"}].def_number(),
                       tab_target[string{"z"}].def_number());

        tab_pose[string{"x"}] = tab_target[string{"x"}];
        tab_pose[string{"y"}] = tab_target[string{"y"}];
        tab_pose[string{"z"}] = tab_target[string{"z"}];
        tab_pose[string{"psi"}] = tab_target[string{"psi"}];

        return {};
    }), false);

    env.assign(string {"pose"}, make_shared<cfunction>([&env](const vallist& args) mutable -> cfunction::result {
        return {env.getvar(string{"__quad_pose"})};
    }), false);

    auto rviz = make_shared<table>();
    env.assign(string {"rviz"}, rviz, false);

    (*rviz)[string {"point"}] = make_shared<lua::rt::cfunction>([this, &parser](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 3 || args[0].type() != "number" || args[1].type() != "number" || args[2].type() != "number") {
            signal.appendTerminal("point requires 3 number arguments");
            return {nil()};
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

                    // make sure the script is only executed once after all highlights are set
                    signal.pauseEval(true);
                    auto modified_tokens = changes.apply(tokens);
                    signal.changeEditorText(QString::fromStdString(get_string(tokens)));
                    signal.highlightTokens(modified_tokens);
                    signal.pauseEval(false);
                }
            });
        return {};
    });

    (*rviz)[string {"line"}] = make_shared<lua::rt::cfunction>([this](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 6
                || args[0].type() != "number" || args[1].type() != "number" || args[2].type() != "number"
                || args[3].type() != "number" || args[4].type() != "number" || args[5].type() != "number") {
            signal.appendTerminal("line requires 6 number arguments");
            return {nil()};
        }
        marker.addLine(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]),
                            get<double>(args[3]), get<double>(args[4]), get<double>(args[5]));
        return {};
    });

    (*rviz)[string {"pose"}] = make_shared<lua::rt::cfunction>([this, &parser](const lua::rt::vallist& args) -> cfunction::result {
        if (args.size() != 4 || !args[0].isnumber() || !args[1].isnumber() || !args[2].isnumber() || !args[3].isnumber()) {
            signal.appendTerminal("pose requires 4 number arguments");
            return {nil()};
        }
        marker.addPose(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]), get<double>(args[3]),
                                    args[0].source.get(), args[1].source.get(), args[2].source.get(), args[3].source.get(),
            [x_source = args[0].source, y_source = args[1].source, z_source = args[2].source, psi_source = args[3].source, this, tokens = parser.tokens, original_psi = get<double>(args[3])](const auto& feedback) mutable {
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
                    if (psi_source && feedback->control_name == "move_psi") {
                        if (const auto& change = psi_source->forceValue(fmod(yaw(feedback->pose.orientation) + original_psi, 2*M_PI)))
                            changes.changes.push_back(*change);
                    }

                    // make sure the script is only executed once after all highlights are set
                    signal.pauseEval(true);
                    auto modified_tokens = changes.apply(tokens);
                    signal.changeEditorText(QString::fromStdString(get_string(tokens)));
                    signal.highlightTokens(modified_tokens);
                    signal.pauseEval(false);
                }
            });
        return {};
    });
}

void LiveScriptInterpreter::run_script(const std::string& script) {
    async = Async([this, script](const Async::cancel_t& cancelled){
        Interpreter interpreter;

        signal.clearTerminal();
        populate_live_env(*interpreter.env, cancelled);
        switch (auto [c, s] = interpreter.dostring(script); c) {
        case Interpreter::ExecResult::ERR_PARSE:
            signal.appendTerminal(QString::fromStdString(s));
            break;
        case Interpreter::ExecResult::ERR_RUNTIME:
            signal.appendTerminal(QString::fromStdString(s));
            break;
        case Interpreter::ExecResult::NOERROR:
            signal.appendTerminal("Info: Execution finished");
        }
    });
}

void LiveScriptInterpreter::populate_live_env(lua::rt::Environment &env, const Async::cancel_t& cancelled) {
    env.assign(string {"print"}, make_shared<lua::rt::cfunction>([this, cancelled](const lua::rt::vallist& args) -> cfunction::result {
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

        signal.highlightTokens(stmt.tokens);

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

        signal.highlightTokens(stmt.tokens);

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

        auto p = quad.get_current_pose();
        table_p t = make_shared<table>();
        (*t)[string {"x"}] = p.position.x;
        (*t)[string {"y"}] = p.position.y;
        (*t)[string {"z"}] = p.position.x;
        (*t)[string {"psi"}] = yaw(p.orientation);

        return {t};
    }), false);

}
