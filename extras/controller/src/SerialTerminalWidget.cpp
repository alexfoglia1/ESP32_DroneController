#include "SerialTerminalWidget.h"
#include <QScrollBar>

SerialTerminalWidget::SerialTerminalWidget(QWidget* parent)
    : QWidget(parent)
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(0);
    mainLayout->setContentsMargins(2, 2, 2, 2);

    QFont terminalFont("Lucida Console", 10);
    terminalFont.setBold(true);

    outputArea = new QTextEdit(this);
    outputArea->setReadOnly(true);
    outputArea->setStyleSheet("background-color: black; color: yellow;");
    outputArea->setFont(terminalFont);
    mainLayout->addWidget(outputArea);

    inputBar = new CommandLineEdit(this);
    inputBar->setStyleSheet("background-color: black; color: yellow;");
    inputBar->setFont(terminalFont);
    mainLayout->addWidget(inputBar);

    connect(inputBar, &QLineEdit::returnPressed, this, &SerialTerminalWidget::handleReturnPressed);

    setLayout(mainLayout);
}

void SerialTerminalWidget::handleReturnPressed()
{
    QString command = inputBar->text();
    if (!command.isEmpty())
    {
        emit commandSent(command);
        inputBar->addToHistory(command);
        inputBar->clear();
    }
}

void SerialTerminalWidget::responseReceived(const QString& response)
{
    outputArea->moveCursor(QTextCursor::End);
    outputArea->insertPlainText(response);
    outputArea->moveCursor(QTextCursor::End);

    QScrollBar* bar = outputArea->verticalScrollBar();
    bar->setValue(bar->maximum());
}
