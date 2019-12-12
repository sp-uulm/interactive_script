#ifndef BLOCKLYWIDGET_H
#define BLOCKLYWIDGET_H

#include <iostream>
#include <QObject>
#include <QPlainTextEdit>
#include <QWebEngineView>
#include <QWebChannel>
#include "interactive_script/blocklybridge.h"

class BlocklyWidget : public QWebEngineView
{
    Q_OBJECT

private:
    BlocklyBridge *bridge;
    QWebChannel *channel;

public:
    explicit BlocklyWidget(QWidget *parent = nullptr);
    void setEditor(QPlainTextEdit *editor);

signals:

public slots:
    void setBlockValue(QString, QString, QString);
};

#endif // BLOCKLYWIDGET_H
