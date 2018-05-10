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

#ifndef FACEFITCONFIG_H
#define FACEFITCONFIG_H

#include <QDialog>

namespace Ui
{
    class FaceFitConfig;
}

namespace ft
{
    /**
     * Form class used to display the configuration window for the face fit utility.
     */
    class FaceFitConfig : public QDialog
    {
        Q_OBJECT

    public:
        /**
         * Class constructor.
         * @param pParent QWidget with the window parent.
         */
        explicit FaceFitConfig(QWidget *pParent = 0);

        /**
         * Class destructor
         */
        virtual ~FaceFitConfig();

		/**
		 * Gets the configured path for the face-fit utility.
		 * @param QString with the path of the face-fit utility.
		 */
		QString getFaceFitPath();

	private slots:
		/**
		* Slot for the Select Path event.
		*/
		void on_actionSelectPath_triggered();

		/**
		 * Slot to handle changes in the path line editor.
		 * @param sFilePath QString with the newly editted utility path.
		 */
		void onPathChanged(QString sFilePath);

		/**
		* Slot to handle the request for help.
		*/
		void onShowHelp();

    private:
        /** Instance of the ui for GUI element access. */
        Ui::FaceFitConfig *ui;
    };
}

#endif // FACEFITCONFIG_H