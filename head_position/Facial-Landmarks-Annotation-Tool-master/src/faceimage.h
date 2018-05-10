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

#ifndef FACE_IMAGE_H
#define FACE_IMAGE_H

#include "facefeature.h"
#include "facefeatureedge.h"

#include <QString>
#include <QPixmap>
#include <QDomDocument>

#include <vector>
#include <string>

namespace ft
{
	/**
	 * Represents the data from one facial image sample in the image dataset, including its
     * annotations (facial features, their connections, and the face emotional label).
	 */
	class FaceImage
    {
	public:
		/**
		 * Class constructor.
		 * @param sFileName String with the file name of the face image. If defined as empty in the
		 * constructor, it can only be later updated by reading from a file.
		 */
		FaceImage(const QString &sFileName = QString());

		/**
		 * Class destructor.
		 */
		virtual ~FaceImage();

		/**
		 * Gets the file name of the face image.
		 * @return QString with the complete file name of the face image.
		 */
		QString fileName() const;

		/**
		 * Sets the file name of the face image.
		 * @param sFileName QString with the new file name for the face image.
		 */
		void setFileName(QString sFileName);

		/**
		 * Adds a new face feature to the face image.
		 * @param iID Integer with the ID of the face feature.
		 * @param x Float with the x coordinate of the face feature.
		 * @param y Float with the y coordinate of the face feature.
		 * @return Instance of a FaceFeature with the new feature created.
		 */
		FaceFeature *addFeature(int iID, float x = 0.0f, float y = 0.0f);

		/**
		 * Gets the face feature at the given index.
		 * @param iIndex Integer with the index of the feature to be obtained.
		 * @return Instance of the face feature at the index or NULL if the index 
		 * is out of bounds.
		 */
		FaceFeature *getFeature(const int iIndex) const;

		/**
		 * Gets all face features in the image.
		 * @return Vector with all instances of face features in the image.
		 */
		std::vector<FaceFeature*> getFeatures() const;

		/**
		 * Removes the face feature at the given index.
		 * @param iIndex Integer with the index of the feature to be removed.
		 * @return Boolean indicating if the face feature was successfully
		 * removed (true) or not (false).
		 */
		bool removeFeature(const int iIndex);

		/**
		* Connects the two given features.
		* @param iIDSource Integer with the ID (index) of the source feature.
		* @param iIDTarget Integer with the ID (index) of the target feature.
		* @return Boolean indicating if the connection was successfully created.
		*/
		bool connectFeatures(int iIDSource, int iIDTarget);

		/**
		* Disconnects the two given features.
		* @param iIDSource Integer with the ID (index) of the source feature.
		* @param iIDTarget Integer with the ID (index) of the target feature.
		* @return Boolean indicating if the connection was successfully removed.
		*/
		bool disconnectFeatures(int iIDSource, int iIDTarget);

		/**
		 * Loads (unserializes) the face image data from the given xml element.
		 * @param oElement QDomElement from where to read the image data (the image node in the xml).
		 * @param sMsgError QString to receive the error message in case the method fails.
		 * @param iNumExpectedFeatures Integer with the number of expected features.
		 * @return Boolean indicating if the loading was successful (true) or if it failed (false).
		 */
		bool loadFromXML(const QDomElement &oElement, QString &sMsgError, int iNumExpectedFeatures);

        /**
         * Saves the face image data into the given xml element.
		 * @param oParent Parent QDomElement to receive the new node of the face image data.
         */
        void saveToXML(QDomElement &oParent) const;

		/**
		 * Loads and returns the image data as a Qt's QPixmap. If an error occur during the loading of
		 * the file, an empty QPixmap is returned instead (that can be checked with QPixmap::isNull()).
		 * @return A QPixmap with the image data, or an empty QPixmap if an error ocurred.
		 */
		QPixmap pixMap() const;

	protected:

		/**
		 * Clears the internal data (like face features).
		 */
		void clear();

	private:

		/** Name of the file with the face image. */
		QString m_sFileName;

		/** Vector of the face features in this face image. */
		std::vector<FaceFeature*> m_vFeatures;

		/** Vector of the connections between face features in this face image. */
		std::vector<FaceFeatureEdge*> m_vConnections;
    };
}

#endif // FACE_IMAGE_H