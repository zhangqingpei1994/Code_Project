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

#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <QPoint>
#include <vector>

namespace ft
{
     /**
      * Static class with utilitary functions of general use.
      */
    class Utils
    {
    protected:
        /**
         * Protected class constructor to prevent this class from being instantiated.
         */
        Utils();

    public:
        /**
         * Shorten the given path by replacing subdirectories with "...". Mainly used
         * for path displaying in the user interface. Exiting file name and drive letter
		 * (in case of Windows) are not replaced. Hence, the minimum string returned
		 * by this method will be the composition of a drive letter at the beginning, 
		 * a file name at the end, and a single "/.../" in between.
         * @param sPath QString with the path to be shortened.
		 * @param iMaxLen Maximum length in characters to trigger the shortening (that is,
		 * the path will be shortened only if its current length exceeds this maximum value).
		 * The default is 100 characters.
         * @return QString with the path shortened and all separators changed to the native OS
		 * standard.
         */
        static QString shortenPath(const QString &sPath, int iMaxLen = 100);

		/**
		 * Reads a points file produced by the face-fit utility.
		 * @return A std::vector with a list of QPoint instances with the points
		 * of facial landmarks fitted to an image, or an empty vector if the reading
		 * failed.
		 */
		static std::vector<QPoint> readFaceFitPointsFile(QString sFileName);
    };
}

#endif // UTILS_H