#ifndef SERIALTERMINALWIDGET_H
#define SERIALTERMINALWIDGET_H

#include <QWidget>
#include <QLineEdit>
#include <QTextEdit>
#include <QVBoxLayout>

#include "CommandLineEdit.h"

class SerialTerminalWidget : public QWidget
{
    Q_OBJECT

public:
    SerialTerminalWidget(QWidget* parent = nullptr);

signals:
    void commandSent(const QString& command);

public slots:
    void responseReceived(const QString& response);

private slots:
    void handleReturnPressed();

private:
    QTextEdit* outputArea;
    CommandLineEdit* inputBar;
};

#endif // SERIALTERMINALWIDGET_H
