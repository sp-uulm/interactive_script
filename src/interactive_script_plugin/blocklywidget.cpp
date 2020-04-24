#include "interactive_script/blocklywidget.h"
#include <ros/package.h>

BlocklyWidget::BlocklyWidget(QWidget *parent) : QWebEngineView(parent)
{
    QString path = "file://"
         + QString::fromStdString(ros::package::getPath("interactive_script_blockly"))
         + "/../quadcopter/index.html";

    std::cout << "Loading blockly URL: " << path.toStdString() << std::endl;

    load(QUrl(path));

    bridge = new BlocklyBridge();
    channel = new QWebChannel(this);

    channel->registerObject("bridge", bridge);
    page()->setWebChannel(channel);
}

void BlocklyWidget::setEditor(QPlainTextEdit *editor) {
    bridge->setEditor(editor);
}

void BlocklyWidget::setBlockValue(QString id, QString field, QString value) {
    emit bridge->set_field_value(id, field, value);
}
