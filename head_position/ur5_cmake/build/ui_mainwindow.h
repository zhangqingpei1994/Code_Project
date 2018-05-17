/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include <qcustomplot.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *label_11;
    QTabWidget *tabWidget;
    QWidget *tab;
    QFrame *frame_3;
    QLabel *label_12;
    QLineEdit *line_Host;
    QLineEdit *line_port;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QPushButton *robot_connect;
    QLabel *connect_info;
    QPushButton *openforce;
    QPushButton *closeforce;
    QFrame *Robot_state;
    QLabel *label;
    QLabel *label_5;
    QLineEdit *tool_actual1;
    QLineEdit *tool_actual2;
    QLineEdit *tool_actual3;
    QLabel *label_16;
    QLabel *label_18;
    QLabel *label_6;
    QLabel *label_7;
    QLineEdit *tool_actual4;
    QLineEdit *tool_actual5;
    QLineEdit *tool_actual6;
    QLabel *label_20;
    QLabel *label_21;
    QFrame *frame_5;
    QFrame *frame_6;
    QLabel *label_17;
    QLineEdit *tool_speed1;
    QLineEdit *tool_speed2;
    QLineEdit *tool_speed3;
    QLineEdit *tool_speed4;
    QLineEdit *tool_speed5;
    QLineEdit *tool_speed6;
    QLabel *label_19;
    QLabel *label_22;
    QLabel *label_23;
    QLabel *label_26;
    QLabel *label_25;
    QLabel *label_24;
    QFrame *frame_7;
    QLabel *label_2;
    QLineEdit *joint_actual1;
    QLineEdit *joint_actual2;
    QLineEdit *joint_actual3;
    QLineEdit *joint_actual4;
    QLineEdit *joint_actual5;
    QLineEdit *joint_actual6;
    QLabel *label_4;
    QLabel *label_27;
    QLabel *label_28;
    QLabel *label_29;
    QLabel *label_30;
    QLabel *label_31;
    QFrame *frame_8;
    QLabel *label_3;
    QLineEdit *joint_speed1;
    QLineEdit *joint_speed2;
    QLineEdit *joint_speed3;
    QLineEdit *joint_speed4;
    QLineEdit *lineEdit_24;
    QLineEdit *joint_speed5;
    QLineEdit *joint_speed6;
    QLabel *label_32;
    QLabel *label_33;
    QLabel *label_34;
    QLabel *label_35;
    QLabel *label_36;
    QLabel *label_37;
    QFrame *frame_4;
    QLineEdit *poscontrol_x;
    QLineEdit *poscontrol_y;
    QLineEdit *poscontrol_z;
    QLineEdit *poscontrol_rx;
    QLineEdit *poscontrol_ry;
    QLineEdit *poscontrol_rz;
    QLineEdit *joint1_control;
    QLineEdit *joint2_control;
    QLineEdit *joint3_control;
    QLineEdit *joint4_control;
    QLineEdit *joint5_control;
    QLineEdit *joint6_control;
    QPushButton *Pos_control;
    QPushButton *Joint_control;
    QLabel *label_38;
    QLabel *label_39;
    QLabel *label_40;
    QLabel *label_41;
    QLabel *label_42;
    QLabel *label_43;
    QLabel *label_44;
    QLabel *label_45;
    QLabel *label_46;
    QLabel *label_47;
    QLabel *label_48;
    QLabel *label_49;
    QFrame *frame_9;
    QLabel *label_50;
    QWidget *tab_2;
    QFrame *frame;
    QLabel *label_8;
    QPushButton *Open_Video;
    QLabel *vedio_frame;
    QFrame *frame_2;
    QLabel *label_9;
    QLabel *label_10;
    QPushButton *Open_1;
    QWidget *tab_3;
    QTextEdit *textEdit;
    QFrame *frame_10;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QFrame *frame_11;
    QLabel *patien_jpg;
    QTextEdit *patient_data;
    QTextEdit *textEdit_3;
    QWidget *tab_4;
    QTextEdit *textEdit_2;
    QCustomPlot *plot;
    QPushButton *level;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1148, 567);
        MainWindow->setIconSize(QSize(10, 24));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(0, 0, 1131, 51));
        QFont font;
        font.setFamily(QStringLiteral("Ubuntu"));
        font.setPointSize(28);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(3);
        label_11->setFont(font);
        label_11->setLayoutDirection(Qt::RightToLeft);
        label_11->setStyleSheet(QLatin1String("border-image: url(:/new/prefix1/picture/1timg.jpeg);\n"
"font: 25 28pt \"Ubuntu\";"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(0, 10, 1131, 491));
        QFont font1;
        font1.setPointSize(10);
        tabWidget->setFont(font1);
        tabWidget->setStyleSheet(QLatin1String("QTabBar::tab{width:80}\n"
"QTabBar::tab{height:40}\n"
"\n"
""));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        frame_3 = new QFrame(tab);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setGeometry(QRect(40, 0, 261, 451));
        frame_3->setStyleSheet(QStringLiteral("background-color: rgb(213, 233, 233);"));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        label_12 = new QLabel(frame_3);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(60, 30, 111, 17));
        line_Host = new QLineEdit(frame_3);
        line_Host->setObjectName(QStringLiteral("line_Host"));
        line_Host->setGeometry(QRect(70, 120, 113, 27));
        line_port = new QLineEdit(frame_3);
        line_port->setObjectName(QStringLiteral("line_port"));
        line_port->setGeometry(QRect(70, 190, 113, 27));
        label_13 = new QLabel(frame_3);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(60, 330, 67, 17));
        label_14 = new QLabel(frame_3);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(20, 120, 31, 17));
        label_15 = new QLabel(frame_3);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(20, 190, 41, 17));
        robot_connect = new QPushButton(frame_3);
        robot_connect->setObjectName(QStringLiteral("robot_connect"));
        robot_connect->setGeometry(QRect(70, 250, 111, 27));
        connect_info = new QLabel(frame_3);
        connect_info->setObjectName(QStringLiteral("connect_info"));
        connect_info->setGeometry(QRect(130, 330, 67, 17));
        openforce = new QPushButton(frame_3);
        openforce->setObjectName(QStringLiteral("openforce"));
        openforce->setGeometry(QRect(20, 400, 99, 27));
        closeforce = new QPushButton(frame_3);
        closeforce->setObjectName(QStringLiteral("closeforce"));
        closeforce->setGeometry(QRect(140, 400, 99, 27));
        Robot_state = new QFrame(tab);
        Robot_state->setObjectName(QStringLiteral("Robot_state"));
        Robot_state->setGeometry(QRect(300, 0, 781, 451));
        Robot_state->setStyleSheet(QStringLiteral("background-color: rgb(213, 233, 233);"));
        Robot_state->setInputMethodHints(Qt::ImhNone);
        Robot_state->setFrameShape(QFrame::StyledPanel);
        Robot_state->setFrameShadow(QFrame::Raised);
        label = new QLabel(Robot_state);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(330, 10, 81, 21));
        label_5 = new QLabel(Robot_state);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(200, 40, 51, 17));
        tool_actual1 = new QLineEdit(Robot_state);
        tool_actual1->setObjectName(QStringLiteral("tool_actual1"));
        tool_actual1->setGeometry(QRect(190, 60, 61, 21));
        tool_actual2 = new QLineEdit(Robot_state);
        tool_actual2->setObjectName(QStringLiteral("tool_actual2"));
        tool_actual2->setGeometry(QRect(280, 60, 61, 21));
        tool_actual3 = new QLineEdit(Robot_state);
        tool_actual3->setObjectName(QStringLiteral("tool_actual3"));
        tool_actual3->setGeometry(QRect(360, 60, 61, 21));
        label_16 = new QLabel(Robot_state);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(10, 60, 171, 17));
        label_18 = new QLabel(Robot_state);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(280, 40, 51, 17));
        label_6 = new QLabel(Robot_state);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(370, 40, 51, 17));
        label_7 = new QLabel(Robot_state);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(450, 40, 67, 17));
        tool_actual4 = new QLineEdit(Robot_state);
        tool_actual4->setObjectName(QStringLiteral("tool_actual4"));
        tool_actual4->setGeometry(QRect(450, 60, 61, 21));
        tool_actual5 = new QLineEdit(Robot_state);
        tool_actual5->setObjectName(QStringLiteral("tool_actual5"));
        tool_actual5->setGeometry(QRect(540, 60, 61, 21));
        tool_actual6 = new QLineEdit(Robot_state);
        tool_actual6->setObjectName(QStringLiteral("tool_actual6"));
        tool_actual6->setGeometry(QRect(640, 60, 61, 21));
        label_20 = new QLabel(Robot_state);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(540, 40, 67, 17));
        label_21 = new QLabel(Robot_state);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setGeometry(QRect(640, 40, 67, 17));
        frame_5 = new QFrame(Robot_state);
        frame_5->setObjectName(QStringLiteral("frame_5"));
        frame_5->setGeometry(QRect(0, 39, 721, 51));
        frame_5->setFrameShape(QFrame::StyledPanel);
        frame_5->setFrameShadow(QFrame::Raised);
        frame_6 = new QFrame(Robot_state);
        frame_6->setObjectName(QStringLiteral("frame_6"));
        frame_6->setGeometry(QRect(0, 90, 721, 61));
        frame_6->setFrameShape(QFrame::StyledPanel);
        frame_6->setFrameShadow(QFrame::Raised);
        label_17 = new QLabel(frame_6);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(10, 20, 171, 17));
        tool_speed1 = new QLineEdit(frame_6);
        tool_speed1->setObjectName(QStringLiteral("tool_speed1"));
        tool_speed1->setGeometry(QRect(190, 30, 61, 21));
        tool_speed2 = new QLineEdit(frame_6);
        tool_speed2->setObjectName(QStringLiteral("tool_speed2"));
        tool_speed2->setGeometry(QRect(280, 30, 61, 21));
        tool_speed3 = new QLineEdit(frame_6);
        tool_speed3->setObjectName(QStringLiteral("tool_speed3"));
        tool_speed3->setGeometry(QRect(360, 30, 61, 21));
        tool_speed4 = new QLineEdit(frame_6);
        tool_speed4->setObjectName(QStringLiteral("tool_speed4"));
        tool_speed4->setGeometry(QRect(450, 30, 61, 21));
        tool_speed5 = new QLineEdit(frame_6);
        tool_speed5->setObjectName(QStringLiteral("tool_speed5"));
        tool_speed5->setGeometry(QRect(540, 30, 61, 21));
        tool_speed6 = new QLineEdit(frame_6);
        tool_speed6->setObjectName(QStringLiteral("tool_speed6"));
        tool_speed6->setGeometry(QRect(640, 30, 61, 21));
        label_19 = new QLabel(frame_6);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(190, 10, 61, 17));
        label_22 = new QLabel(frame_6);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setGeometry(QRect(280, 10, 61, 17));
        label_23 = new QLabel(frame_6);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setGeometry(QRect(360, 10, 61, 17));
        label_26 = new QLabel(frame_6);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setGeometry(QRect(450, 10, 61, 17));
        label_25 = new QLabel(frame_6);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setGeometry(QRect(540, 10, 61, 17));
        label_24 = new QLabel(frame_6);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setGeometry(QRect(640, 10, 61, 17));
        frame_7 = new QFrame(Robot_state);
        frame_7->setObjectName(QStringLiteral("frame_7"));
        frame_7->setGeometry(QRect(0, 150, 721, 61));
        frame_7->setFrameShape(QFrame::StyledPanel);
        frame_7->setFrameShadow(QFrame::Raised);
        label_2 = new QLabel(frame_7);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 20, 81, 17));
        joint_actual1 = new QLineEdit(frame_7);
        joint_actual1->setObjectName(QStringLiteral("joint_actual1"));
        joint_actual1->setGeometry(QRect(190, 30, 61, 21));
        joint_actual2 = new QLineEdit(frame_7);
        joint_actual2->setObjectName(QStringLiteral("joint_actual2"));
        joint_actual2->setGeometry(QRect(280, 30, 61, 21));
        joint_actual3 = new QLineEdit(frame_7);
        joint_actual3->setObjectName(QStringLiteral("joint_actual3"));
        joint_actual3->setGeometry(QRect(360, 30, 61, 21));
        joint_actual4 = new QLineEdit(frame_7);
        joint_actual4->setObjectName(QStringLiteral("joint_actual4"));
        joint_actual4->setGeometry(QRect(460, 30, 61, 21));
        joint_actual5 = new QLineEdit(frame_7);
        joint_actual5->setObjectName(QStringLiteral("joint_actual5"));
        joint_actual5->setGeometry(QRect(550, 30, 61, 21));
        joint_actual6 = new QLineEdit(frame_7);
        joint_actual6->setObjectName(QStringLiteral("joint_actual6"));
        joint_actual6->setGeometry(QRect(650, 30, 61, 21));
        label_4 = new QLabel(frame_7);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(200, 10, 31, 17));
        label_27 = new QLabel(frame_7);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setGeometry(QRect(290, 10, 31, 17));
        label_28 = new QLabel(frame_7);
        label_28->setObjectName(QStringLiteral("label_28"));
        label_28->setGeometry(QRect(370, 10, 31, 17));
        label_29 = new QLabel(frame_7);
        label_29->setObjectName(QStringLiteral("label_29"));
        label_29->setGeometry(QRect(470, 10, 31, 17));
        label_30 = new QLabel(frame_7);
        label_30->setObjectName(QStringLiteral("label_30"));
        label_30->setGeometry(QRect(560, 10, 31, 17));
        label_31 = new QLabel(frame_7);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setGeometry(QRect(660, 10, 31, 17));
        frame_8 = new QFrame(Robot_state);
        frame_8->setObjectName(QStringLiteral("frame_8"));
        frame_8->setGeometry(QRect(0, 210, 721, 61));
        frame_8->setFrameShape(QFrame::StyledPanel);
        frame_8->setFrameShadow(QFrame::Raised);
        label_3 = new QLabel(frame_8);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 20, 91, 17));
        joint_speed1 = new QLineEdit(frame_8);
        joint_speed1->setObjectName(QStringLiteral("joint_speed1"));
        joint_speed1->setGeometry(QRect(190, 30, 61, 21));
        joint_speed2 = new QLineEdit(frame_8);
        joint_speed2->setObjectName(QStringLiteral("joint_speed2"));
        joint_speed2->setGeometry(QRect(280, 30, 61, 21));
        joint_speed3 = new QLineEdit(frame_8);
        joint_speed3->setObjectName(QStringLiteral("joint_speed3"));
        joint_speed3->setGeometry(QRect(370, 30, 61, 21));
        joint_speed4 = new QLineEdit(frame_8);
        joint_speed4->setObjectName(QStringLiteral("joint_speed4"));
        joint_speed4->setGeometry(QRect(460, 30, 61, 21));
        lineEdit_24 = new QLineEdit(frame_8);
        lineEdit_24->setObjectName(QStringLiteral("lineEdit_24"));
        lineEdit_24->setGeometry(QRect(580, 60, 61, 21));
        joint_speed5 = new QLineEdit(frame_8);
        joint_speed5->setObjectName(QStringLiteral("joint_speed5"));
        joint_speed5->setGeometry(QRect(560, 30, 61, 21));
        joint_speed6 = new QLineEdit(frame_8);
        joint_speed6->setObjectName(QStringLiteral("joint_speed6"));
        joint_speed6->setGeometry(QRect(650, 30, 61, 21));
        label_32 = new QLabel(frame_8);
        label_32->setObjectName(QStringLiteral("label_32"));
        label_32->setGeometry(QRect(200, 10, 51, 17));
        label_33 = new QLabel(frame_8);
        label_33->setObjectName(QStringLiteral("label_33"));
        label_33->setGeometry(QRect(290, 10, 51, 17));
        label_34 = new QLabel(frame_8);
        label_34->setObjectName(QStringLiteral("label_34"));
        label_34->setGeometry(QRect(380, 10, 51, 17));
        label_35 = new QLabel(frame_8);
        label_35->setObjectName(QStringLiteral("label_35"));
        label_35->setGeometry(QRect(470, 10, 51, 17));
        label_36 = new QLabel(frame_8);
        label_36->setObjectName(QStringLiteral("label_36"));
        label_36->setGeometry(QRect(570, 10, 51, 20));
        label_37 = new QLabel(frame_8);
        label_37->setObjectName(QStringLiteral("label_37"));
        label_37->setGeometry(QRect(660, 10, 51, 17));
        frame_4 = new QFrame(Robot_state);
        frame_4->setObjectName(QStringLiteral("frame_4"));
        frame_4->setGeometry(QRect(-10, 320, 731, 131));
        frame_4->setFrameShape(QFrame::StyledPanel);
        frame_4->setFrameShadow(QFrame::Raised);
        poscontrol_x = new QLineEdit(frame_4);
        poscontrol_x->setObjectName(QStringLiteral("poscontrol_x"));
        poscontrol_x->setGeometry(QRect(10, 40, 81, 27));
        poscontrol_y = new QLineEdit(frame_4);
        poscontrol_y->setObjectName(QStringLiteral("poscontrol_y"));
        poscontrol_y->setGeometry(QRect(110, 40, 81, 27));
        poscontrol_z = new QLineEdit(frame_4);
        poscontrol_z->setObjectName(QStringLiteral("poscontrol_z"));
        poscontrol_z->setGeometry(QRect(210, 40, 81, 27));
        poscontrol_rx = new QLineEdit(frame_4);
        poscontrol_rx->setObjectName(QStringLiteral("poscontrol_rx"));
        poscontrol_rx->setGeometry(QRect(10, 90, 81, 27));
        poscontrol_ry = new QLineEdit(frame_4);
        poscontrol_ry->setObjectName(QStringLiteral("poscontrol_ry"));
        poscontrol_ry->setGeometry(QRect(110, 90, 81, 27));
        poscontrol_rz = new QLineEdit(frame_4);
        poscontrol_rz->setObjectName(QStringLiteral("poscontrol_rz"));
        poscontrol_rz->setGeometry(QRect(210, 90, 81, 27));
        joint1_control = new QLineEdit(frame_4);
        joint1_control->setObjectName(QStringLiteral("joint1_control"));
        joint1_control->setGeometry(QRect(410, 40, 81, 27));
        joint2_control = new QLineEdit(frame_4);
        joint2_control->setObjectName(QStringLiteral("joint2_control"));
        joint2_control->setGeometry(QRect(510, 40, 81, 27));
        joint3_control = new QLineEdit(frame_4);
        joint3_control->setObjectName(QStringLiteral("joint3_control"));
        joint3_control->setGeometry(QRect(620, 40, 81, 27));
        joint4_control = new QLineEdit(frame_4);
        joint4_control->setObjectName(QStringLiteral("joint4_control"));
        joint4_control->setGeometry(QRect(410, 90, 81, 27));
        joint5_control = new QLineEdit(frame_4);
        joint5_control->setObjectName(QStringLiteral("joint5_control"));
        joint5_control->setGeometry(QRect(510, 90, 81, 27));
        joint6_control = new QLineEdit(frame_4);
        joint6_control->setObjectName(QStringLiteral("joint6_control"));
        joint6_control->setGeometry(QRect(620, 90, 81, 27));
        Pos_control = new QPushButton(frame_4);
        Pos_control->setObjectName(QStringLiteral("Pos_control"));
        Pos_control->setGeometry(QRect(300, 40, 91, 27));
        Joint_control = new QPushButton(frame_4);
        Joint_control->setObjectName(QStringLiteral("Joint_control"));
        Joint_control->setGeometry(QRect(300, 90, 101, 27));
        label_38 = new QLabel(frame_4);
        label_38->setObjectName(QStringLiteral("label_38"));
        label_38->setGeometry(QRect(20, 10, 61, 17));
        label_39 = new QLabel(frame_4);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setGeometry(QRect(120, 10, 67, 17));
        label_40 = new QLabel(frame_4);
        label_40->setObjectName(QStringLiteral("label_40"));
        label_40->setGeometry(QRect(220, 10, 67, 17));
        label_41 = new QLabel(frame_4);
        label_41->setObjectName(QStringLiteral("label_41"));
        label_41->setGeometry(QRect(20, 70, 81, 17));
        label_42 = new QLabel(frame_4);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setGeometry(QRect(120, 70, 81, 17));
        label_43 = new QLabel(frame_4);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setGeometry(QRect(210, 70, 81, 17));
        label_44 = new QLabel(frame_4);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setGeometry(QRect(420, 20, 67, 17));
        label_45 = new QLabel(frame_4);
        label_45->setObjectName(QStringLiteral("label_45"));
        label_45->setGeometry(QRect(520, 20, 67, 17));
        label_46 = new QLabel(frame_4);
        label_46->setObjectName(QStringLiteral("label_46"));
        label_46->setGeometry(QRect(630, 20, 67, 17));
        label_47 = new QLabel(frame_4);
        label_47->setObjectName(QStringLiteral("label_47"));
        label_47->setGeometry(QRect(420, 70, 67, 17));
        label_48 = new QLabel(frame_4);
        label_48->setObjectName(QStringLiteral("label_48"));
        label_48->setGeometry(QRect(520, 70, 67, 17));
        label_49 = new QLabel(frame_4);
        label_49->setObjectName(QStringLiteral("label_49"));
        label_49->setGeometry(QRect(630, 70, 67, 17));
        frame_9 = new QFrame(Robot_state);
        frame_9->setObjectName(QStringLiteral("frame_9"));
        frame_9->setGeometry(QRect(0, 270, 721, 51));
        frame_9->setFrameShape(QFrame::StyledPanel);
        frame_9->setFrameShadow(QFrame::Raised);
        label_50 = new QLabel(frame_9);
        label_50->setObjectName(QStringLiteral("label_50"));
        label_50->setGeometry(QRect(330, 20, 101, 17));
        frame_6->raise();
        frame_5->raise();
        label->raise();
        label_5->raise();
        tool_actual1->raise();
        tool_actual2->raise();
        tool_actual3->raise();
        label_16->raise();
        label_18->raise();
        label_6->raise();
        label_7->raise();
        tool_actual4->raise();
        tool_actual5->raise();
        tool_actual6->raise();
        label_20->raise();
        label_21->raise();
        frame_7->raise();
        frame_8->raise();
        frame_4->raise();
        frame_9->raise();
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        frame = new QFrame(tab_2);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(40, 0, 501, 451));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        label_8 = new QLabel(frame);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(140, 410, 151, 21));
        Open_Video = new QPushButton(frame);
        Open_Video->setObjectName(QStringLiteral("Open_Video"));
        Open_Video->setGeometry(QRect(340, 410, 99, 27));
        vedio_frame = new QLabel(frame);
        vedio_frame->setObjectName(QStringLiteral("vedio_frame"));
        vedio_frame->setGeometry(QRect(0, 0, 501, 411));
        vedio_frame->raise();
        label_8->raise();
        Open_Video->raise();
        frame_2 = new QFrame(tab_2);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(540, -10, 541, 461));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        label_9 = new QLabel(frame_2);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(-20, 0, 571, 391));
        label_9->setStyleSheet(QString::fromUtf8("border-image: url(:/new/prefix1/picture/\345\233\276\347\211\2071.jpg)"));
        label_10 = new QLabel(frame_2);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(180, 410, 171, 20));
        Open_1 = new QPushButton(frame_2);
        Open_1->setObjectName(QStringLiteral("Open_1"));
        Open_1->setGeometry(QRect(430, 410, 99, 27));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        textEdit = new QTextEdit(tab_3);
        textEdit->setObjectName(QStringLiteral("textEdit"));
        textEdit->setGeometry(QRect(623, -3, 411, 451));
        frame_10 = new QFrame(tab_3);
        frame_10->setObjectName(QStringLiteral("frame_10"));
        frame_10->setGeometry(QRect(-50, 0, 641, 451));
        frame_10->setFrameShape(QFrame::StyledPanel);
        frame_10->setFrameShadow(QFrame::Raised);
        pushButton = new QPushButton(frame_10);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(530, 40, 99, 27));
        pushButton_2 = new QPushButton(frame_10);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(530, 150, 99, 27));
        pushButton_3 = new QPushButton(frame_10);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));
        pushButton_3->setGeometry(QRect(530, 260, 99, 27));
        frame_11 = new QFrame(frame_10);
        frame_11->setObjectName(QStringLiteral("frame_11"));
        frame_11->setGeometry(QRect(50, 0, 471, 451));
        frame_11->setFrameShape(QFrame::StyledPanel);
        frame_11->setFrameShadow(QFrame::Raised);
        patien_jpg = new QLabel(frame_11);
        patien_jpg->setObjectName(QStringLiteral("patien_jpg"));
        patien_jpg->setGeometry(QRect(0, 120, 161, 171));
        patient_data = new QTextEdit(frame_11);
        patient_data->setObjectName(QStringLiteral("patient_data"));
        patient_data->setGeometry(QRect(160, 90, 311, 341));
        textEdit_3 = new QTextEdit(frame_11);
        textEdit_3->setObjectName(QStringLiteral("textEdit_3"));
        textEdit_3->setGeometry(QRect(0, 0, 471, 51));
        patient_data->raise();
        patien_jpg->raise();
        textEdit_3->raise();
        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QStringLiteral("tab_4"));
        textEdit_2 = new QTextEdit(tab_4);
        textEdit_2->setObjectName(QStringLiteral("textEdit_2"));
        textEdit_2->setGeometry(QRect(0, 20, 231, 411));
        plot = new QCustomPlot(tab_4);
        plot->setObjectName(QStringLiteral("plot"));
        plot->setGeometry(QRect(340, 20, 741, 411));
        level = new QPushButton(tab_4);
        level->setObjectName(QStringLiteral("level"));
        level->setGeometry(QRect(230, 190, 99, 27));
        tabWidget->addTab(tab_4, QString());
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1148, 31));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(3);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        label_11->setText(QApplication::translate("MainWindow", "\347\273\217\351\242\205\347\243\201\345\210\272\346\277\200\346\231\272\350\203\275\345\256\232\344\275\215\345\257\274\350\210\252\347\263\273\347\273\237", 0));
#ifndef QT_NO_TOOLTIP
        tabWidget->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p><br/></p></body></html>", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        tabWidget->setWhatsThis(QApplication::translate("MainWindow", "<html><head/><body><p>1111</p></body></html>", 0));
#endif // QT_NO_WHATSTHIS
        label_12->setText(QApplication::translate("MainWindow", "Robot connect", 0));
        label_13->setText(QApplication::translate("MainWindow", "\350\277\236\346\216\245\347\212\266\346\200\201:", 0));
        label_14->setText(QApplication::translate("MainWindow", "\344\270\273\346\234\272", 0));
        label_15->setText(QApplication::translate("MainWindow", "\347\253\257\345\217\243", 0));
        robot_connect->setText(QApplication::translate("MainWindow", "robot_connect", 0));
        connect_info->setText(QApplication::translate("MainWindow", "TextLabel", 0));
        openforce->setText(QApplication::translate("MainWindow", "force_open", 0));
        closeforce->setText(QApplication::translate("MainWindow", "force_close", 0));
        label->setText(QApplication::translate("MainWindow", "Robot state", 0));
        label_5->setText(QApplication::translate("MainWindow", "X(mm)", 0));
        label_16->setText(QApplication::translate("MainWindow", "\345\267\245\345\205\267\345\234\250Base\345\235\220\346\240\207\347\263\273\344\270\213\344\275\215\347\275\256", 0));
        label_18->setText(QApplication::translate("MainWindow", "Y(mm)", 0));
        label_6->setText(QApplication::translate("MainWindow", "Z(mm)", 0));
        label_7->setText(QApplication::translate("MainWindow", "Rx(rad)", 0));
        label_20->setText(QApplication::translate("MainWindow", "Ry(rad)", 0));
        label_21->setText(QApplication::translate("MainWindow", "Rz(rad)", 0));
        label_17->setText(QApplication::translate("MainWindow", "\345\267\245\345\205\267\345\234\250Base\345\235\220\346\240\207\347\263\273\344\270\213\351\200\237\345\272\246", 0));
        label_19->setText(QApplication::translate("MainWindow", "X(mm/s)", 0));
        label_22->setText(QApplication::translate("MainWindow", "Y(mm/s)", 0));
        label_23->setText(QApplication::translate("MainWindow", "Z(mm/s)", 0));
        label_26->setText(QApplication::translate("MainWindow", "Rx(rad/s)", 0));
        label_25->setText(QApplication::translate("MainWindow", "Ry(rad/s)", 0));
        label_24->setText(QApplication::translate("MainWindow", "Rz(rad/s)", 0));
        label_2->setText(QApplication::translate("MainWindow", "\345\220\204\345\205\263\350\212\202\350\247\222\345\272\246", 0));
        label_4->setText(QApplication::translate("MainWindow", "j0(\302\260\357\274\211", 0));
        label_27->setText(QApplication::translate("MainWindow", "j1(\302\260\357\274\211", 0));
        label_28->setText(QApplication::translate("MainWindow", "j2(\302\260\357\274\211", 0));
        label_29->setText(QApplication::translate("MainWindow", "j3(\302\260\357\274\211", 0));
        label_30->setText(QApplication::translate("MainWindow", "j4(\302\260\357\274\211", 0));
        label_31->setText(QApplication::translate("MainWindow", "j5(\302\260\357\274\211", 0));
        label_3->setText(QApplication::translate("MainWindow", "\345\220\204\345\205\263\350\212\202\350\247\222\351\200\237\345\272\246", 0));
        label_32->setText(QApplication::translate("MainWindow", "j0(\302\260/s\357\274\211", 0));
        label_33->setText(QApplication::translate("MainWindow", "j1(\302\260/s\357\274\211", 0));
        label_34->setText(QApplication::translate("MainWindow", "j2(\302\260/s\357\274\211", 0));
        label_35->setText(QApplication::translate("MainWindow", "j3(\302\260/s\357\274\211", 0));
        label_36->setText(QApplication::translate("MainWindow", "j4(\302\260/s\357\274\211", 0));
        label_37->setText(QApplication::translate("MainWindow", "j5(\302\260/s\357\274\211", 0));
        Pos_control->setText(QApplication::translate("MainWindow", "Pos_control", 0));
        Joint_control->setText(QApplication::translate("MainWindow", "Joint_control", 0));
        label_38->setText(QApplication::translate("MainWindow", "Pos_x(m)", 0));
        label_39->setText(QApplication::translate("MainWindow", "Pos_y(m)", 0));
        label_40->setText(QApplication::translate("MainWindow", "Pos_z(m)", 0));
        label_41->setText(QApplication::translate("MainWindow", "Pos_Rx(rad)", 0));
        label_42->setText(QApplication::translate("MainWindow", "Pos_Ry(rad)", 0));
        label_43->setText(QApplication::translate("MainWindow", "Pos_Rz(rad)", 0));
        label_44->setText(QApplication::translate("MainWindow", "Joint1(\302\260)", 0));
        label_45->setText(QApplication::translate("MainWindow", "Joint2(\302\260)", 0));
        label_46->setText(QApplication::translate("MainWindow", "Joint3(\302\260)", 0));
        label_47->setText(QApplication::translate("MainWindow", "Joint4(\302\260)", 0));
        label_48->setText(QApplication::translate("MainWindow", "Joint5(\302\260)", 0));
        label_49->setText(QApplication::translate("MainWindow", "Joint6(\302\260)", 0));
        label_50->setText(QApplication::translate("MainWindow", "Robot control", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "\346\234\272\346\242\260\350\207\202\347\212\266\346\200\201", 0));
        label_8->setText(QApplication::translate("MainWindow", "Monitor the window", 0));
        Open_Video->setText(QApplication::translate("MainWindow", "Open_Video", 0));
        vedio_frame->setText(QApplication::translate("MainWindow", "TextLabel", 0));
        label_9->setText(QApplication::translate("MainWindow", "TextLabel", 0));
        label_10->setText(QApplication::translate("MainWindow", "Treatment area window", 0));
        Open_1->setText(QApplication::translate("MainWindow", "Open_1", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "\346\262\273\347\226\227\347\212\266\346\200\201", 0));
        textEdit->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#00e4ff;\">\347\227\205\346\203\205\347\261\273\345\210\253               \345\210\272\346\277\200\345\274\272\345\272\246 </span>                   <span style=\" color:#00e2ff;\"> \345\210\272\346\277\200\346\227\266\351\227\264</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">\347\231\253\347\227\253                  1HZ/1200/70%      20min/5\344\270\252\350\277\236\347\273\255\345\221\250\346\234\237</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px;"
                        " margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">PD                    25HZ/1800               \350\277\236\347\273\25510\345\244\251</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">\345\244\261\347\234\240          2.0T/0.5Hz/\346\257\217\346\227\245\344\270\200\346\254\241  \345\210\272\346\277\200\345\217\214\344\276\247\351\242\235\351\241\266\345\214\272/\346\257\217\351\203\250\344\275\21530\346\254\241</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">\345\207\217\350\275\273\350\207\252\346\235\200\350\247\202\345\277\265     20%\350\277\220\345\212\250\351\230\210\345\200\274   \345\267\246\345\211\215\351\242\235\345\217\266\347\232\256\350\264\250</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">\346\210\222\347\230\276</p>\n"
"<p style=\""
                        " margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">\345\271\277\346\263\233\346\200\247\347\204\246\350\231\221\351\232\234\347\242\215</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">\346\212\221\351\203\201\347\227\207</p></body></html>", 0));
        pushButton->setText(QApplication::translate("MainWindow", "\351\207\207\351\233\206\346\202\243\350\200\205\344\277\241\346\201\257", 0));
        pushButton_2->setText(QApplication::translate("MainWindow", "\346\202\243\350\200\205\345\214\271\351\205\215", 0));
        pushButton_3->setText(QApplication::translate("MainWindow", "PushButton", 0));
        patien_jpg->setText(QApplication::translate("MainWindow", "\346\202\243\350\200\205\345\244\264\345\203\217", 0));
        textEdit_3->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:18pt; color:#00ffff;\">\344\272\272\350\204\270\350\257\206\345\210\253\347\273\223\346\236\234</span></p></body></html>", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "\346\202\243\350\200\205\346\225\260\346\215\256", 0));
        textEdit_2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">30\344\276\213\345\270\225\351\207\221\346\243\256\346\202\243\350\200\205\345\210\272\346\277\200\345\220\216\350\272\253\344\275\223\345\220\204\351\241\271\346\214\207\346\240\207\345\271\263\345\235\207\345\200\274</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:12pt;\"><br /></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0;"
                        " text-indent:0px; font-size:12pt;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">\346\237\220\346\202\243\350\200\205\350\277\2213\344\270\252\346\234\210\345\210\272\346\277\200\346\203\205\345\206\265\345\217\212\346\257\217\345\244\251\350\272\253\344\275\223\346\214\207\346\240\207</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:12pt;\"><br /></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:12pt;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">\345\220\204\347\261\273\346\202\243\350\200\205\345\210\272\346\277\200\345\220\216\350\272"
                        "\253\344\275\223\346\237\220\344\272\233\346\214\207\346\240\207\345\217\230\346\215\242\346\203\205\345\206\265</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:12pt;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">............................................................</span></p></body></html>", 0));
        level->setText(QApplication::translate("MainWindow", "\350\241\200\347\263\226\345\220\253\351\207\217", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "\346\225\260\346\215\256\345\210\206\346\236\220", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
