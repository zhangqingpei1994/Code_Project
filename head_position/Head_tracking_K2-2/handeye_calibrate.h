#ifndef HANDEYE_CALIBRATE_H
#define HANDEYE_CALIBRATE_H

#include <QObject>

class Handeye_Calibrate : public QObject
{
    Q_OBJECT
public:
    explicit Handeye_Calibrate(QObject *parent = 0);

signals:

public slots:
};

#endif // HANDEYE_CALIBRATE_H