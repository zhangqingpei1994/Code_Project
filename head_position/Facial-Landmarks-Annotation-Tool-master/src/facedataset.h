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

#ifndef FACEDATASET_H
#define FACEDATASET_H

#include "faceimage.h"
#include "facefeature.h"

#include <QDomDocument>

#include <vector>

namespace ft
{
	/**
	 * Represents a dataset of facial images that can be annotated with facial landmarks and
	 * prototypical emotional labels.
	 */
	class FaceDataset
	{
	public:
		/**
		 * Class constructor.
		 */
		FaceDataset();

		/**
		 * Class destructor.
		 */
		virtual ~FaceDataset();

		/**
		 * Returns the number of face samples in the dataset.
		 * @return Integer with the number of face samples in this dataset.
		 */
		int size() const;

		/**
		 * Loads (unserializes) the instance from the given text file in the YAML format
		 * (YAML Ain't Markup Language - http://en.wikipedia.org/wiki/YAML).
		 * @param sFileName QString with the name of the file to read the data from.
		 * @param sMsgError QString to receive the error message in case the method fails.
		 * @return Boolean indicating if the loading was successful (true) of failed (false).
		 */
		bool loadFromFile(const QString &sFileName, QString &sMsgError);

        /**
         * Saves (serializes) the instance to the given file in the YAML format
		 * (YAML Ain't Markup Language - http://en.wikipedia.org/wiki/YAML).
         * @param sFileName QString with the name of the file to write the data to.
		 * @param sMsgError QString to receive the error message in case the method fails.
		 * @return Boolean indicating if the saving was successful (true) of failed (false).
         */
        bool saveToFile(const QString &sFileName, QString &sMsgError) const;

        /**
         * Clear the face annotation dataset.
         */
        void clear();

		/**
		 * Gets the face image for the given index. The index must be in the range [0, count - 1],
		 * where count is the number of face images in the dataset.
		 * @param iIndex Integer with the index of the image file to load.
		 * @return Pointer to a FaceImage with the face image data. If the method fails, NULL is returned. 
		 */
		FaceImage* getImage(const int iIndex) const;

		/**
		 * Adds a new image to the face annotation dataset. All other data (landmarks, connections, etc)
		 * are created with default values.
		 * @param sFileName QString with the path and filename of the image file to be added to the dataset.
		 * @return Pointer to a FaceImage with the new image added to the dataset or NULL if failed. 
		 */
		FaceImage* addImage(const QString &sFileName);

		/**
		 * Removes an image from the face annotation dataset. All other data (landmarks, connections, etc)
		 * are also removed.
		 * @param iIndex Integer with the index of the image to remove.
		 * @return Boolean indicating if the image was removed (true) or not (false, in case
		 * the given index is out of range).
		 */
		bool removeImage(const int iIndex);

		/**
		 * Queries the number of facial features in the dataset (applicable to all images).
		 * @return Integer with the number of face features in the dataset.
		 */
		int numFeatures() const;

		/**
		 * Updates the number of facial features in the dataset (applicable to all images).
		 * @param iNumFeats Integer with the new number of facial features for the dataset.
		 */
		void setNumFeatures(int iNumFeats);

		/**
		 * Adds a new feature to the face dataset. A new feature is added to all
		 * face images in the dataset in the same coordinates.
		 * @param iID Integer with the identifier of the face feature.
		 * @param x Float with the x coordinate for the face features.
		 * @param y Float with the y coordinate for the face features.
		 */
		void addFeature(int iID, float x = 0.0f, float y = 0.0f);

		/**
		 * Removes an existing feature from the face dataset. The feature is removed from all
		 * face images in the dataset.
		 * @param iIndex Integer with the index of the feature to remove.
		 * @return Boolean indicating if the feature was successfully removed (true) or not (false).
		 */
		bool removeFeature(const int iIndex);

		/**
		* Connects the two given features.
		* @param iIDSource Integer with the ID of the source feature.
		* @param iIDTarget Integer with the ID of the target feature.
		* @return Boolean indicating if the connection was successfully created.
		*/
		bool connectFeatures(int iIDSource, int iIDTarget);

		/**
		* Disconnects the two given features.
		* @param iIDSource Integer with the ID of the source feature.
		* @param iIDTarget Integer with the ID of the target feature.
		* @return Boolean indicating if the connection was successfully removed.
		*/
		bool disconnectFeatures(int iIDSource, int iIDTarget);

		/**
		 * Gets the list of face features in the given image index.
		 * @param iIndex Integer with the index of the image to query the face features.
		 * @param A vector of FaceFeature instances with the face features in the image. It returns
		 * an empty vector if the given index is invalid.
		 */
		std::vector<FaceFeature*> getImageFeatures(const int iIndex);

	private:

		/** Vector of sample face images. */
		std::vector<FaceImage*> m_vSamples;

		/** Number of face features in the dataset (i.e. applicable to all images). */
		int m_iNumFeatures;
	};
}

#endif // FACEDATASET_H
