/*
 * Copyright (C) 2016 Luiz Carlos Vieira (http://www.luiz.vieira.nom.br)
 *
 * This file is part of FLAT.
 *
 * FLAT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FLAT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "application.h"
#include <typeinfo>
#include <QtGlobal>
#include <QDateTime>
#include <QFileInfo>
#include <QDebug>
#include <QMessageBox>
#include <QSettings>
#include <QDir>
#include <QStandardPaths>
#include <QFileInfo>

#include <iostream>
#include <stdexcept>

using namespace std;

// +-----------------------------------------------------------
ft::FtApplication::FtApplication(int argc, char* argv[]): QApplication(argc, argv)
{
    m_pMainWindow = NULL;

	// Information used to store the program settings
    QCoreApplication::setOrganizationName("Flat");
    QCoreApplication::setOrganizationDomain("https://github.com/luigivieira/Facial-Landmarks-Annotation-Tool.git");
    QCoreApplication::setApplicationName("Data");

	QString sAppFile = QCoreApplication::applicationFilePath();
	QString sDocPath = QDir::toNativeSeparators(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)) + QDir::separator();
	QFileInfo oFile = QFileInfo(sAppFile);
	QString sLogFile = QString("%1%2.log").arg(sDocPath).arg(oFile.baseName());

	m_oLogFile.open(qPrintable(sLogFile), ios::app);
	if (!m_oLogFile.is_open())
	{
		cerr << QString("Could not open the file [%1] for writing. No log will be created.").arg(sLogFile).toStdString();
		exit(-1);
	}
	
	qInstallMessageHandler(&ft::FtApplication::handleLogOutput);
	
	qDebug() << QCoreApplication::applicationFilePath().toStdString().c_str() << "started.";
}

// +-----------------------------------------------------------
ft::FtApplication::~FtApplication()
{
	qDebug() << QCoreApplication::applicationFilePath().toStdString().c_str() << "ended.";

	if (m_oLogFile.is_open())
	{
		m_oLogFile.flush();
		m_oLogFile.close();
	}
}

// +-----------------------------------------------------------
bool ft::FtApplication::notify(QObject* pReceiver, QEvent* pEvent)
{
    try
	{
		// Retransmit the event notification
        return QApplication::notify(pReceiver, pEvent);
    }
    catch (std::exception &e)
	{
        qFatal("Exception %s sending event [%s] to object [%s] (%s)",
            e.what(), typeid(*pEvent).name(), qPrintable(pReceiver->objectName()), typeid(*pReceiver).name());
    }
    catch (...)
	{
        qFatal("Exception sending event [%s] to object [%s] (%s)",
            typeid(*pEvent).name(), qPrintable(pReceiver->objectName()), typeid(*pReceiver).name());
    }

    return false;
}

// +-----------------------------------------------------------
void ft::FtApplication::handleLogOutput(QtMsgType eType, const QMessageLogContext &oContext, const QString &sMsg) {
	QString sNow = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

	// Only include context information if it exists (when compiled in debug)
	QString sDebugInfo;
	if(oContext.line != 0)
	{
		QString sSource = QFileInfo(QFile(oContext.file).fileName()).fileName();
		sDebugInfo = QString(" [%1:%2, %3] ").arg(sSource).arg(oContext.line).arg(oContext.function);
	}
	else
		sDebugInfo = " ";

	switch (eType)
	{
		case QtDebugMsg:
			FtApplication::instance()->m_oLogFile << qPrintable(sNow) << qPrintable(sDebugInfo) << "DEBUG: " << qPrintable(sMsg) << endl;
			FtApplication::instance()->m_oLogFile.flush();
			break;

		case QtWarningMsg:
			FtApplication::instance()->m_oLogFile << qPrintable(sNow) << qPrintable(sDebugInfo) << "WARNING: " << qPrintable(sMsg) << endl;
			FtApplication::instance()->m_oLogFile.flush();
			break;

		case QtCriticalMsg:
			FtApplication::instance()->m_oLogFile << qPrintable(sNow) << qPrintable(sDebugInfo) << "CRITICAL: " << qPrintable(sMsg) << endl;
			FtApplication::instance()->m_oLogFile.flush();
			break;

		case QtFatalMsg:
			QApplication::beep();
			QMessageBox::critical(NULL, qApp->translate("Main", "Runtime Error"), qApp->translate("Main", "A severe exception happened and the application must terminate."), QMessageBox::Ok);

			cerr << qPrintable(sNow) << qPrintable(sDebugInfo) << "FATAL: " << qPrintable(sMsg) << endl;
			FtApplication::instance()->m_oLogFile << qPrintable(sNow) << qPrintable(sDebugInfo) << "FATAL:" << qPrintable(sMsg) << endl;
			exit(-2);
	}
}

// +-----------------------------------------------------------
ft::FtApplication* ft::FtApplication::instance()
{
	return (FtApplication *) qApp;
}

// +-----------------------------------------------------------
void ft::FtApplication::showStatusMessage(const QString &sMsg, const int iTimeout)
{
	FtApplication::instance()->emit statusMessageShown(sMsg, iTimeout);
}