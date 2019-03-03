#ifndef TESTINGCONSOLE_H
#define TESTINGCONSOLE_H

#include <QWidget>
#include <QString>
#include <QSerialPort>
#include <QTimer>
#include "console.h"
#include <deque>

namespace Ui {
class TestingConsole;
}

class TestingConsole : public QWidget
{
    Q_OBJECT

public:
    TestingConsole(QWidget *parent = 0, QSerialPort *port = nullptr);
    ~TestingConsole();
    void parseLine(const QString &line);
    Console *getConsole();

private:
    void readNXBytesCharacteristic(const QString &line, int numberOfValues, std::deque<int> &numbers);

signals:
    void writeData(const QByteArray &data);

public slots:
    void updateEncoderWrtitingCounter();

private:
    Ui::TestingConsole *ui;
    QSerialPort *m_serial = nullptr;
    QTimer *m_timer;
    uint encoderTimesWritten;
    int encoderPosition;
    uint discToothNumber;
    double discPerimeter;

    std::deque<int> extractedValues;
};

#endif // TESTINGCONSOLE_H
