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

#include "mainwindow.h"
#include "application.h"

using namespace ft;

// +-----------------------------------------------------------
int main(int argc, char *argv[])
{
	FtApplication oApp(argc, argv);
	
	MainWindow oMainWindow;
	QObject::connect(FtApplication::instance(), SIGNAL(statusMessageShown(const QString &, const int)), &oMainWindow, SLOT(showStatusMessage(const QString &, const int)));

	oMainWindow.show();
	return oApp.exec();
}
