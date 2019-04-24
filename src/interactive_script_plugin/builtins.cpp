#include "interactive_script/builtins.h"
#include "interactive_script/interactive_script_plugin.h"

#include <sstream>
#include <string>
#include <variant>
#include <QString>

using namespace lua::rt;
using namespace std;

void VisualizationInterpreter::run_script() {
    LuaParser parser;
    const auto result = parser.parse(gui->editor->toPlainText().toStdString());

    if (holds_alternative<string>(result)) {
        gui->terminal->setPlainText(QString::fromStdString("Error: " + get<string>(result)));
        parse_result.reset();
    } else {
        parse_result = get<LuaChunk>(result);

        auto env = make_shared<lua::rt::Environment>(nullptr);

        env->populate_stdlib();
        populate_visualization_env(*env, parser);

        gui->terminal->setPlainText("");
        lua::rt::ASTEvaluator eval;
        if (auto eval_result = parse_result->accept(eval, env); holds_alternative<string>(eval_result)) {
            gui->terminal->setPlainText(QString::fromStdString("Error: " + get<string>(eval_result)));
        } else {
            marker.commit();
        }

        env->clear();
    }
}

void VisualizationInterpreter::populate_visualization_env(Environment& env, LuaParser& parser) {
    env.assign(string {"print"}, make_shared<lua::rt::cfunction>([this](const lua::rt::vallist& args) -> lua::rt::vallist {
        stringstream ss;
        for (int i = 0; i < static_cast<int>(args.size()) - 1; ++i) {
            ss << args[static_cast<unsigned>(i)].to_string() << "\t";
        }
        if (!args.empty()) {
            ss << args.back().to_string();
        }
        gui->terminal->appendPlainText(QString::fromStdString(ss.str()));
        return {};
    }), false);

    env.assign(string {"moveTo"}, make_shared<cfunction>([this, &env](const vallist& args) mutable -> vallist {
        if (args.size() == 3 && args[0].isnumber() && args[1].isnumber() && args[2].isnumber()) {
            val pose = env.getvar(string{"__quad_pose"});
            if (pose.type() != "table") {
                pose = make_shared<table>();
                env.assign(string{"__quad_pose"}, pose, false);
            }

            table& p = *get<table_p>(pose);

            marker.addLine(p[string{"x"}].def_number(), p[string{"y"}].def_number(), p[string{"z"}].def_number(),
                           get<double>(args[0]), get<double>(args[1]), get<double>(args[2]));

            p[string{"x"}] = args[0];
            p[string{"y"}] = args[1];
            p[string{"z"}] = args[2];

            return {};
        }

        gui->terminal->appendPlainText("invalid args to moveTo(x, y, z)");
        return {nil()};
    }), false);

    env.assign(string {"pose"}, make_shared<cfunction>([&env](const vallist& args) mutable -> vallist {
        return {env.getvar(string{"__quad_pose"})};
    }), false);

    auto rviz = make_shared<table>();
    env.assign(string {"rviz"}, rviz, false);

    (*rviz)[string {"point"}] = make_shared<lua::rt::cfunction>([this, &parser](const lua::rt::vallist& args) -> lua::rt::vallist {
        if (args.size() != 3 || args[0].type() != "number" || args[1].type() != "number" || args[2].type() != "number") {
            gui->terminal->appendPlainText("point requires 3 number arguments");
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
                    changes.apply(tokens);
                    emit signal.changeEditorText(QString::fromStdString(get_string(tokens)));
                }
            });
        return {};
    });

    (*rviz)[string {"line"}] = make_shared<lua::rt::cfunction>([this](const lua::rt::vallist& args) -> lua::rt::vallist {
        if (args.size() != 6
                || args[0].type() != "number" || args[1].type() != "number" || args[2].type() != "number"
                || args[3].type() != "number" || args[4].type() != "number" || args[5].type() != "number") {
            gui->terminal->appendPlainText("line requires 6 number arguments");
            return {nil()};
        }
        marker.addLine(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]),
                            get<double>(args[3]), get<double>(args[4]), get<double>(args[5]));
        return {};
    });

    (*rviz)[string {"pose"}] = make_shared<lua::rt::cfunction>([this, &parser](const lua::rt::vallist& args) -> lua::rt::vallist {
        if (args.size() != 4 || !args[0].isnumber() || !args[1].isnumber() || !args[2].isnumber() || !args[3].isnumber()) {
            gui->terminal->appendPlainText("pose requires 4 number arguments");
            return {nil()};
        }
        marker.addPose(get<double>(args[0]), get<double>(args[1]), get<double>(args[2]), get<double>(args[3]),
                                    args[0].source.get(), args[1].source.get(), args[2].source.get(), args[3].source.get(),
            [x_source = args[0].source, y_source = args[1].source, z_source = args[2].source, psi_source = args[3].source, this, tokens = parser.tokens](const auto& feedback) mutable {
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
                        if (const auto& change = psi_source->forceValue(yaw(feedback->pose.orientation)))
                            changes.changes.push_back(*change);
                    }
                    changes.apply(tokens);
                    emit signal.changeEditorText(QString::fromStdString(get_string(tokens)));
                }
            });
        return {};
    });
}
