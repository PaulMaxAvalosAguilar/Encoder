#include "testingconsole.h"
#include "ui_testingconsole.h"
#include <QtMath>
#include <QDebug>
#include <iostream>

TestingConsole::TestingConsole(QWidget *parent, QSerialPort *port) :
    QWidget(parent),
    ui(new Ui::TestingConsole),
    m_serial(port),
    m_timer(new QTimer(this)),
    encoderTimesWritten(0),
    encoderPosition(0),
    extractedValues()
{
    ui->setupUi(this);

    discToothNumber = 10;
    discPerimeter = 120;

    QString physicalPropertiesText;

    physicalPropertiesText+= "Tooth number: ";
    physicalPropertiesText+= QString::number(discToothNumber);
    ui->toothLabel->setText(physicalPropertiesText);

    physicalPropertiesText = "Turn(mm): ";
    physicalPropertiesText += QString::number(discPerimeter);
    ui->turnmmLabel->setText(physicalPropertiesText);

    connect(m_timer, SIGNAL(timeout()), this, SLOT(updateEncoderWrtitingCounter()));
}

TestingConsole::~TestingConsole()
{
    delete ui;
}

void TestingConsole::parseLine(const QString &line)
{
    ui->m_console->putData(line);

    QByteArray sendMessage = "";
    static int configurations = 0;
    static int advertisings = 0;
    if(line == "InitializingBluetooth\n"){
        configurations = 0;
        advertisings = 0;
        encoderTimesWritten = 0;
        ui->m_console->clear();

        sendMessage = "CMD\n";
        emit writeData(sendMessage);
        ui->m_console->putData("Testing initialized\n");


    }else if(line.contains("SF") ||
             line.contains("PZ") ||
             line.contains("SR") ||
             line.contains("SS") ||
             line.contains("PS") ||
             line.contains("PC") ||
             line.contains("SN") ||
             line.contains("LS")){
        configurations++;
        QString config;

        if(line == "LS\n"){
            sendMessage =
                    "180F\n"
                    "  2A19,0018,V\n"
                    "  2A19,0019,C\n"
                    "11223344556677889900AABBCCDDEEFF\n"
                    "  010203040506070809000A0B0C0D1FFF,001A,02,20\n"
                    "  010203040506070809000A0B0C0D0E0F,001B,10,20\n"
                    "  010203040506070809000A0B0C0D0E0F,001C,10,20\n"
                    "  111213141516171819101A1B1C1D1E1F,001D,02,20\n"
                    "END\n";
        }else{
            sendMessage = "AOK\n";
        }
        emit writeData(sendMessage);

        ui->m_console->putData(config.append("Configuration: ").append(
                               QString::number(configurations).append(
                                   "/10\n")));
        if(configurations == 10){
            ui->m_console->putData("ConfigurationTests: Passed\n");
        }


    }else if(line == "A\n"){
        advertisings++;

        sendMessage = "AOK\n";
        emit writeData(sendMessage);

        if(advertisings == 1){
            sendMessage = "Connected\n";
            emit writeData(sendMessage);
            sendMessage = "Connection End\n";
            emit writeData(sendMessage);
            //Start advertising
            sendMessage = "WC,001C,0100\n";
            emit writeData(sendMessage);

        }else{
            ui->m_console->putData("Connectivity Test: Passed\n");

        }

    }else if(line == "U\n"){
        sendMessage = "AOK\n";
        emit writeData(sendMessage);
    }else if(line.contains("SHW")){
        sendMessage = "AOK\n";
        emit writeData(sendMessage);
        if(line.contains("001B")){

            const int numberOfValues = 8;

            readNXBytesCharacteristic(line, numberOfValues, extractedValues);

            static int firstTimeTimer = 1;
            if(firstTimeTimer == 1){
                firstTimeTimer = 0;
                m_timer->start(1000);
            }
            encoderTimesWritten++;

            sendMessage = "AOK\n";
            emit writeData(sendMessage);

            QString outputData;
            for(int i = 0; i < numberOfValues; i ++){
                int num = extractedValues.at(i);
                encoderPosition = num;
                outputData.append((num >= 0)? "+":"-");
                outputData.append(QString::number((num >= 0)? num: -num));
                ui->posvalLabel->setText(outputData);
                outputData.clear();
            }
            extractedValues.clear();
        }
        /*
        }else if(line.contains("0018")){

            const int numberOfValues = 1;
            const int byteSize = 1;
            readNXBytesCharacteristic(line, numberOfValues, byteSize, signs,numbers);

            QString completeData;
            for(int i = 0; i < numberOfValues; i ++){
                completeData.append(signs.at(i));
                num = numbers.at(i).toUInt(nullptr,16);
                completeData.append(QString::number(num));
                completeData.append("%");

                ui->battvalLabel->setText(completeData);
            }
        }
       */
    }
}

Console *TestingConsole::getConsole()
{
    return ui->m_console;
}

void TestingConsole::readNXBytesCharacteristic(const QString &line, int numberOfValues, std::deque<int> &numbers)
{
    int valueBytesSize = 2;
    auto encoderStringIterator = line.begin()+9;

    QString hexSignsString = "";
    uint signs = 0;
    QString dataStringValue = "";
    int value;

    for(auto iterator = encoderStringIterator; iterator < encoderStringIterator + 2;
        iterator++){
        hexSignsString.append(*iterator);
    }

    signs = hexSignsString.toUInt(nullptr, 16);

    encoderStringIterator += 2;
    for(int i = 0; i < numberOfValues; i ++){

        uint8_t sign = ( signs & (1 << i));

        for(auto iterator = encoderStringIterator; iterator < encoderStringIterator + (valueBytesSize*2);
            iterator++){
            dataStringValue.append(*iterator);
        }

        value = (sign)? -dataStringValue.toInt(nullptr, 16): dataStringValue.toInt(nullptr,16);
        qDebug()<< value;

        numbers.push_back(value);

        encoderStringIterator += (valueBytesSize*2);
        dataStringValue.clear();
    }
}

void TestingConsole::updateEncoderWrtitingCounter()
{
    static int lastTimesCalled = 0;
    static int lastEncoderPosition = 0;
    static int pulses = discToothNumber;
    static double turnmm = discPerimeter;

    uint lectures = encoderTimesWritten-lastTimesCalled;
    int pulsesPerSecond = encoderPosition-lastEncoderPosition;
    double metersPerSecond =
            pulsesPerSecond * (turnmm / (pulses*2)) *((double)1/1000);


    ui->lecturesvalLabel->setText(QString::number(lectures));
    ui->pulsvalLabel->setText(QString::number(qFabs(pulsesPerSecond)));
    ui->metersvalLabel->setText(QString::number(metersPerSecond));
    lastTimesCalled = encoderTimesWritten;
    lastEncoderPosition = encoderPosition;
    m_timer->start(1000);
}
