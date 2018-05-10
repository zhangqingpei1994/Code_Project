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

#ifndef APPLICATION_H
#define APPLICATION_H

#include <QApplication>
#include <QMainWindow>
#include <QObject>

#include <fstream>

namespace ft
{
	/**
	 * Custom application class, used to intercept message notifications in the project applications.
	 */
	class FtApplication: public QApplication
	{
		Q_OBJECT
	public:
		/**
		* Class constructor.
		* @param argc Number of arguments received from the command line.
		* @param argv Array of char pointers with the arguments received from the command line.
		*/
		FtApplication(int argc, char* argv[]);

		/**
		* Class destructor.
		*/
		virtual ~FtApplication();

		/**
		 * Displays a text message in the main window status bar.
		 * @param sMsg QString with the message to be displayed.
		 * @param iTimeout Integer with the number of miliseconds
		 * by which the message will be displayed. The default is 5000
		 * (i.e. 5 seconds).
		 */
		static void showStatusMessage(const QString &sMsg, const int iTimeout = 5000);

		/**
		 * Handles the notification of messages in the application event loop.
		 * @param pReceiver Pointer to the QObject that shall receive the message.
		 * @param pEvent Pointer to the QEvent with the message information.
		 */
        bool notify(QObject* pReceiver, QEvent* pEvent);

		/**
		 * Gets the default instance for this application (from the qApp variable simply type-casted).
		 * @return Pointer to the default instance of the FtApplication.
		 */
		static FtApplication* instance();

	signals:
		/**
		 * Signal indicating that a status message was requested to be displayed on the
		 * application main window.
		 * @param sMsg QString with the message to be displayed.
		 * @param iTimeout Integer with the number of miliseconds
		 * by which the message will be displayed.
		 */
		void statusMessageShown(const QString &sMsg, const int iTimeout);

	protected:
		/**
		 * Log and exception message handler for application events.
 		 * @param eType QtMsgType enum value with the type of the log event.
		 * @param oContext QMessageLogContext instance with information on where the event happened (function, line, etc)
		 * @param sMsg QString instance with the event message.
		 */
		static void handleLogOutput(QtMsgType eType, const QMessageLogContext& oContext, const QString& sMsg);

	private:
		/** File stream used to log application messages. */
		std::ofstream m_oLogFile;
		
		/** Instance of the main window used with this application. */
		QMainWindow *m_pMainWindow;
	};
}

#endif // APPLICATION_H
