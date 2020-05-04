/*
  Copyright 2016 Lucas Walter
*/

#include "interactive_script/interactive_script_plugin.h"
#include "interactive_script/builtins.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QTimer>
#include <QTextCursor>

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

    qRegisterMetaType<TokenMessage>();
    qRegisterMetaType<SourceChangeMessage>();
    qRegisterMetaType<QTextCharFormat>();

    connect(ui_.editor, &QPlainTextEdit::textChanged,
            this, &InteractiveScriptGui::onTextChanged);

    connect(ui_.run_button, &QPushButton::clicked,
            this, &InteractiveScriptGui::onRunScriptClicked);

    connect(&vis.signal, &SignalObject::changeEditorText,
            this, &InteractiveScriptGui::onChangeEditorText);
    connect(&vis.signal, &SignalObject::applySourceChanges,
            this, &InteractiveScriptGui::onApplySourceChanges);
    connect(&vis.signal, &SignalObject::clearTerminal,
            this, &InteractiveScriptGui::onClearTerminal);
    connect(&vis.signal, &SignalObject::appendTerminal,
            this, &InteractiveScriptGui::onAppendTerminal);
    connect(&vis.signal, &SignalObject::highlightTokens,
            this, &InteractiveScriptGui::onHighlightTokens);
    connect(&vis.signal, &SignalObject::removeFormatting,
            this, &InteractiveScriptGui::onRemoveFormatting);

    connect(&live.signal, &SignalObject::changeEditorText,
            this, &InteractiveScriptGui::onChangeEditorText);
    connect(&live.signal, &SignalObject::applySourceChanges,
            this, &InteractiveScriptGui::onApplySourceChanges);
    connect(&live.signal, &SignalObject::clearTerminal,
            this, &InteractiveScriptGui::onClearTerminal);
    connect(&live.signal, &SignalObject::appendTerminal,
            this, &InteractiveScriptGui::onAppendTerminal);
    connect(&live.signal, &SignalObject::highlightTokens,
            this, &InteractiveScriptGui::onHighlightTokens);

    if(!connect(&vis.signal, &SignalObject::setBlockValue,
                ui_.blockly_widget, &BlocklyWidget::setBlockValue))
        ROS_ERROR("could not connect signal setBlockValue");

    if(!connect(&vis.signal, &SignalObject::highlightBlock,
                ui_.blockly_widget, &BlocklyWidget::highlightBlock))
        ROS_ERROR("could not connect signal highlightBlock");

    if(!connect(&vis.signal, &SignalObject::highlightField,
                ui_.blockly_widget, &BlocklyWidget::highlightField))
        ROS_ERROR("could not connect signal highlightField");


    setlocale(LC_ALL, "C");
    onTextChanged();

    // set the editor into which the blockly widget writes the generated code
    ui_.blockly_widget->setEditor(ui_.editor);

    QTimer* timer = new QTimer(this);
    timer->setInterval(1000/60.0);
    timer->start();
    connect(timer, &QTimer::timeout,
            this,  &InteractiveScriptGui::updateMarkerInterface);

    // recalculate the visualization once per second
    QTimer* timer2 = new QTimer(this);
    timer2->setInterval(1000/15.0);
    timer2->start();
    connect(timer2, &QTimer::timeout,
            this,  &InteractiveScriptGui::updateTf);
}

void InteractiveScriptGui::shutdownPlugin()
{
  is_shutdown = true;
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

void InteractiveScriptGui::execute_vis() {
    if (!is_shutdown) {
        string s = ui_.editor->toPlainText().toStdString();
        vis.run_script(s, settings);
    }
}

void InteractiveScriptGui::removeFormatting() {
    QTextCursor cursor(ui_.editor->document());
    cursor.select(QTextCursor::Document);
    cursor.setCharFormat(QTextCharFormat());
}

void InteractiveScriptGui::onHighlightTokens(TokenMessage tokens, QTextCharFormat fmt) {
    eval_paused = true;

    QTextCursor cursor(ui_.editor->document());

    for (const auto& t : tokens) {
        cout << "highlight: " << t << endl;

        cursor.setPosition(t.pos, QTextCursor::MoveAnchor);
        cursor.setPosition(t.pos+t.length, QTextCursor::KeepAnchor);
        cursor.setCharFormat(fmt);
    }

    eval_paused = false;
}

void InteractiveScriptGui::onRemoveFormatting() {
    // remove any formatting
    eval_paused = true;
    removeFormatting();
    ui_.blockly_widget->removeHighlights();
    eval_paused = false;
}

void InteractiveScriptGui::onChangeEditorText(QString s) {
    ui_.editor->setPlainText(s);
}

struct EditorSCVisitor : lua::rt::ApplySCVisitor {
    bool apply_changes(QTextCursor& cursor, QTextCharFormat format = QTextCharFormat()) {
        // sort the collected changes according to their position
        std::sort(changes.begin(), changes.end(), [](const auto& a, const auto& b){
            return a.token.pos > b.token.pos;
        });

        bool changed = false;

        // apply the changes from back to front
        for (const auto& sc : changes) {
            cursor.setPosition(sc.token.pos, QTextCursor::MoveAnchor);
            cursor.setPosition(sc.token.pos + sc.token.length, QTextCursor::KeepAnchor);

            if (QString s = QString::fromStdString(sc.replacement); cursor.selectedText() != s) {
                cursor.insertText(s, format);
                changed = true;
            }
        }

        // delete list of changes
        changes.clear();

        return changed;
    }
};

void InteractiveScriptGui::onApplySourceChanges(SourceChangeMessage msg, QTextCharFormat fmt) {
    EditorSCVisitor vis;
    msg->accept(vis);

    QTextCursor cursor(ui_.editor->document());

    eval_paused = true;
    bool changed = vis.apply_changes(cursor, fmt);
    eval_paused = false;

    if (changed)
        execute_vis();
}

void InteractiveScriptGui::onClearTerminal() {
    ui_.terminal->setPlainText("");
}

void InteractiveScriptGui::onAppendTerminal(QString s) {
    ui_.terminal->appendPlainText(s);
}

void InteractiveScriptGui::onTextChanged() {
    if (!eval_paused) {

        // remove any formatting
        eval_paused = true;
        removeFormatting();
        eval_paused = false;

//        execute_vis();
    }
}

void InteractiveScriptGui::onRunScriptClicked() {
    live.run_script(ui_.editor->toPlainText().toStdString(), settings);
}

void InteractiveScriptGui::updateTf() {
    if (!eval_paused) {
        execute_vis();
    }

    if (settings.print_performance_statistics) {
        ROS_INFO_STREAM_THROTTLE(1, "Total time [us]: " << vis.ps.total.count() << endl
                                 << "- tokenize [us]: " << vis.ps.tokenize.count() << endl
                                 << "- parse    [us]: " << vis.ps.parse.count() << endl
                                 << "- lib/env  [us]: " << vis.ps.create_env.count() << endl
                                 << "- execute  [us]: " << (vis.ps.execute-vis.ps.marker_interface).count() << endl
                                 << "- marker   [us]: " << vis.ps.marker_interface.count() << endl
                                 << "- apply sc [us]: " << vis.ps.source_changes.count() << endl);
    }
}

bool InteractiveScriptGui::hasConfiguration() const {
  return true;
}

void InteractiveScriptGui::triggerConfiguration() {
    ConfigDialog config;
    config.ui->move_arrows->setChecked(settings.move_markers);
    config.ui->display_dependencies->setChecked(settings.click_for_dependency_trace);
    config.ui->dependency_colors->setChecked(settings.dependency_trace_colors);
    config.ui->performance_statistics->setChecked(settings.print_performance_statistics);

    if (config.exec() == QDialog::Accepted) {
        settings.move_markers = config.ui->move_arrows->isChecked();
        settings.click_for_dependency_trace = config.ui->display_dependencies->isChecked();
        settings.dependency_trace_colors = config.ui->dependency_colors->isChecked();
        settings.print_performance_statistics = config.ui->performance_statistics->isChecked();
    }
  // Usually used to open a dialog to offer the user a set of configuration
}

}  // namespace rqt_example_cpp
PLUGINLIB_EXPORT_CLASS(interactive_script_plugin::InteractiveScriptGui, rqt_gui_cpp::Plugin)
